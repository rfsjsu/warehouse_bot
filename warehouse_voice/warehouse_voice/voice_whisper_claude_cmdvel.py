import os, json, time, threading
import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from faster_whisper import WhisperModel  # :contentReference[oaicite:2]{index=2}
from anthropic import Anthropic         # :contentReference[oaicite:3]{index=3}


SYSTEM_PROMPT = """You are a robot command parser.
Return ONLY valid JSON, no prose, no markdown.
Schema:
{
  "intent": "forward"|"backward"|"left"|"right"|"stop"|"none",
  "speed": number,        // 0.0..0.6 (m/s) for forward/backward
  "turn_rate": number,    // 0.0..1.5 (rad/s) for left/right
  "duration": number      // 0.0..3.0 seconds
}
Rules:
- If user asks to move forward, intent="forward".
- If user asks to stop, intent="stop" and duration=0.
- If unclear, intent="none".
- Prefer duration 1.0 sec if not specified.
- Prefer speed 0.2, turn_rate 0.8 if not specified.
"""


def clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


class VoiceWhisperClaudeCmdVel(Node):
    def __init__(self):
        super().__init__("voice_whisper_claude_cmdvel")

        # ---- ROS params ----
        self.declare_parameter("wake_word", "robot")     # Only process command with "robot"
        self.declare_parameter("require_wake_word", True)
        self.declare_parameter("min_text_len", 6)        # Ignore too short sentence
        
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("device_index", -1)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_sec", 2.0)          # short chunk STT
        self.declare_parameter("whisper_model", "small.en")  # short or medium English: small.en / medium.en
        self.declare_parameter("whisper_device", "cpu")    # if gpu "cuda"
        self.declare_parameter("whisper_compute", "int8")  # if cpu int8
        self.declare_parameter("language", "en")           # English Language
        self.declare_parameter("claude_model", "claude-sonnet-4-5")  # Based on account setting
        self.declare_parameter("min_conf_chars", 3)
        self.declare_parameter("cooldown_sec", 1.0)

        self.wake_word = str(self.get_parameter("wake_word").value).lower()
        self.require_wake_word = bool(self.get_parameter("require_wake_word").value)
        self.min_text_len = int(self.get_parameter("min_text_len").value)
        
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.device_index = int(self.get_parameter("device_index").value)
        self.sr = int(self.get_parameter("sample_rate").value)
        self.chunk_sec = float(self.get_parameter("chunk_sec").value)
        self.whisper_model_name = self.get_parameter("whisper_model").value
        self.whisper_device = self.get_parameter("whisper_device").value
        self.whisper_compute = self.get_parameter("whisper_compute").value
        self.lang = self.get_parameter("language").value
        self.claude_model = self.get_parameter("claude_model").value
        self.min_conf_chars = int(self.get_parameter("min_conf_chars").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # ---- Whisper ----
        self.get_logger().info(f"Loading faster-whisper: {self.whisper_model_name} ({self.whisper_device}/{self.whisper_compute})")
        self.whisper = WhisperModel(self.whisper_model_name, device=self.whisper_device, compute_type=self.whisper_compute)

        # ---- Claude ----
        api_key = os.environ.get("ANTHROPIC_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("ANTHROPIC_API_KEY env var not set")
        self.claude = Anthropic(api_key=api_key)

        self._lock = threading.Lock()
        self._cooldown_until = 0.0

        self.get_logger().info("Listening (English). Say: 'go forward', 'stop', 'turn left' ...")

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _publish(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.pub.publish(msg)

    def _do_motion(self, intent: str, speed: float, turn_rate: float, duration: float):
        # safey clamp
        speed = clamp(speed, 0.0, 0.6)
        turn_rate = clamp(turn_rate, 0.0, 1.5)
        duration = clamp(duration, 0.0, 3.0)

        if intent == "stop":
            self._publish(0.0, 0.0)
            self.get_logger().info("CMD: STOP")
            return

        lin, ang = 0.0, 0.0
        if intent == "forward":
            lin = +speed
        elif intent == "backward":
            lin = -speed
        elif intent == "left":
            ang = +turn_rate
        elif intent == "right":
            ang = -turn_rate
        else:
            return

        self.get_logger().info(f"CMD: {intent} speed={speed:.2f} turn={turn_rate:.2f} dur={duration:.2f}s")

        t_end = time.time() + duration
        rate = 20.0
        dt = 1.0 / rate
        while rclpy.ok() and time.time() < t_end:
            self._publish(lin, ang)
            time.sleep(dt)
        self._publish(0.0, 0.0)

    def _claude_parse(self, text: str) -> dict:
        # Let clause parse only intention (cost/delay minimal)
        resp = self.claude.messages.create(
            model=self.claude_model,
            max_tokens=120,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": text}],
        )
        # SDK response content is usually text bock
        out = ""
        for block in resp.content:
            if getattr(block, "type", None) == "text":
                out += block.text
        out = out.strip()
        
        # ---- strip markdown code fences if present ----
        # Examples:
        # ```json
        # { ... }
        # ```
        if out.startswith("```"):
            # remove first fence line
            lines = out.splitlines()
            if len(lines) >= 1 and lines[0].startswith("```"):
                lines = lines[1:]
            # remove last fence line if present
            if len(lines) >= 1 and lines[-1].strip().startswith("```"):
                lines = lines[:-1]
            out = "\n".join(lines).strip()

        # also handle accidental leading 'json' token
        if out.lower().startswith("json"):
            out = out[4:].strip()
        

        if not out:
            raise RuntimeError("Claude returned empty text (check API key, model name, or network)")
        self.get_logger().info(f"Claude raw: {out[:200]}")
        return json.loads(out)

    def _handle_text(self, text: str):
        text = (text or "").strip().lower()
        if not text:
            return

        # Ignore too short sentence (Example: "you", "pause")
        if len(text) < self.min_text_len:
            return

        # Local Instant Stop: Not through Claude, stop (safety)
        if "stop" in text or "halt" in text or "freeze" in text:
            self.get_logger().info("Local STOP trigger")
            self._publish(0.0, 0.0)
            return

        # Without wake word, not taking as command (Preventing Noise or TV dialge)
        if self.require_wake_word:
            if self.wake_word not in text:
                return
            # Only use after wake word (Example: "robot go forward" -> "go forward")
            text = text.split(self.wake_word, 1)[-1].strip()
            if not text:
                return

        # Cool Down (Preventing continuing burst)
        with self._lock:
            now = time.time()
            if now < self._cooldown_until:
                return
            self._cooldown_until = now + self.cooldown_sec

        self.get_logger().info(f"Heard(cmd): {text}")

        # Claude parsing -> Execute motion
        try:
            cmd = self._claude_parse(text)
            intent = cmd.get("intent", "none")
            speed = cmd.get("speed", 0.2)
            turn = cmd.get("turn_rate", 0.8)
            dur = cmd.get("duration", 1.0)
        except Exception as e:
            self.get_logger().error(f"Claude parse failed: {e}")
            return

        threading.Thread(target=self._do_motion, args=(intent, speed, turn, dur), daemon=True).start()


    def _record_chunk(self) -> np.ndarray:
        frames = int(self.sr * self.chunk_sec)
        audio = sd.rec(frames, samplerate=self.sr, channels=1, dtype="float32",
                       device=None if self.device_index < 0 else self.device_index)
        sd.wait()
        return audio.reshape(-1)

    def _stt(self, audio_f32: np.ndarray) -> str:
        # faster-whisper expects float32 numpy array OK
        segments, info = self.whisper.transcribe(audio_f32, language=self.lang, vad_filter=True)
        text = " ".join(seg.text.strip() for seg in segments).strip()
        return text

    def _loop(self):
        while rclpy.ok():
            try:
                audio = self._record_chunk()
                text = self._stt(audio)
                if text:
                    self._handle_text(text)
            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")
                time.sleep(0.2)


def main():
    rclpy.init()
    node = VoiceWhisperClaudeCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
