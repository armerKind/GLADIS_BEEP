import io
import json
import unittest
from pathlib import Path

from PIL import Image

from beep_eyes.gemini import GeminiError, GeminiVisionClient


FIXTURE = Path(__file__).parent / "fixtures" / "eyes_response.json"


def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (32, 24), "gray").save(output, format="JPEG")
    return output.getvalue()


class GeminiVisionClientTests(unittest.TestCase):
    def test_request_uses_header_key_images_and_json_schema(self):
        expected = json.loads(FIXTURE.read_text())
        seen = {}
        credential = "unit-test-sentinel"

        def transport(url, headers, body, timeout):
            seen.update(url=url, headers=headers, body=json.loads(body), timeout=timeout)
            return {"candidates": [{"content": {"parts": [{"text": json.dumps(expected)}]}}]}

        client = GeminiVisionClient(credential, model="gemini-test", transport=transport)
        result = client.analyze({"packet_id": "p1"}, jpeg(), jpeg())
        self.assertEqual(result["schema_version"], "1.0")
        self.assertNotIn(credential, seen["url"])
        self.assertEqual(seen["headers"]["x-goog-api-key"], credential)
        config = seen["body"]["generationConfig"]
        self.assertEqual(config["responseMimeType"], "application/json")
        self.assertIn("responseJsonSchema", config)
        parts = seen["body"]["contents"][0]["parts"]
        self.assertEqual(len([part for part in parts if "inlineData" in part]), 2)

    def test_rejects_non_json_model_output(self):
        def transport(*_):
            return {"candidates": [{"content": {"parts": [{"text": "not json"}]}}]}

        client = GeminiVisionClient(api_key="x", transport=transport)
        with self.assertRaisesRegex(GeminiError, "not valid JSON"):
            client.analyze({}, jpeg(), jpeg())

    def test_rejects_missing_candidate(self):
        client = GeminiVisionClient(api_key="x", transport=lambda *_: {})
        with self.assertRaisesRegex(GeminiError, "no candidate"):
            client.analyze({}, jpeg(), jpeg())

    def test_repairs_one_locally_invalid_compact_response(self):
        calls = []
        base = {
            "schema_version": "1.0", "scene_summary": "Room.", "changes": [],
            "entities": [], "hazards": [], "attention_type": "none", "attention_id": "",
            "attention_reason": "None.", "goal": "Observe.", "confidence": 0.8,
            "uncertainty": [], "escalate": False, "escalation_reason": "",
        }
        invalid = {**base, "steps": [{"skill": "advance", "reason": "Closer.", "argument": "", "amount": 1.0}]}
        valid = {**base, "steps": [{"skill": "observe", "reason": "Wait.", "argument": "", "amount": 0}]}

        def transport(_url, _headers, body, _timeout):
            calls.append(json.loads(body))
            payload = invalid if len(calls) == 1 else valid
            return {"candidates": [{"content": {"parts": [{"text": json.dumps(payload)}]}}]}

        result = GeminiVisionClient(api_key="x", transport=transport).analyze({}, jpeg(), jpeg())
        self.assertEqual(result["plan"]["steps"][0]["skill"], "observe")
        self.assertEqual(len(calls), 2)
        self.assertIn("violated this local contract", calls[1]["contents"][0]["parts"][0]["text"])


if __name__ == "__main__":
    unittest.main()
