"""No-network contract and failure checks for the bounded live provider."""
import io
import json
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

from examples.nebius_recovery.provider import (
    DEFAULT_MODEL, ENDPOINT, MAX_RESPONSE_BYTES, FixtureProvider, NebiusProvider,
    ProviderError, validate_decision,
)


DECISION = {"action": "reach", "target_id": "workpiece", "reason": "Fresh target"}
OBSERVATION = {"event": "initial", "scene": {"target_id": "workpiece",
               "position": [0.32, -0.31, 1.13], "version": 1, "age_s": 0.0},
               "retries_remaining": 1}


class Response(io.BytesIO):
    headers = {"x-request-id": "req-test"}


def completion(**updates):
    value = {"id": "chat-test", "model": DEFAULT_MODEL,
             "choices": [{"finish_reason": "stop", "message": {
                 "role": "assistant", "content": json.dumps(DECISION)}}],
             "usage": {"prompt_tokens": 100, "completion_tokens": 20, "total_tokens": 120}}
    value.update(updates)
    return value


class ProviderTests(unittest.TestCase):
    def provider(self, value=None):
        provider = NebiusProvider(api_key="test-secret", model=DEFAULT_MODEL)
        self.calls = []

        def opened(req, **kwargs):
            self.calls.append((req, kwargs))
            return Response(json.dumps(value if value is not None else completion()).encode())

        provider._open = opened
        return provider

    def test_live_request_contract_and_sanitized_evidence(self):
        decision, evidence = self.provider().decide(OBSERVATION)
        self.assertEqual(decision, DECISION)
        req, options = self.calls[0]
        payload = json.loads(req.data)
        self.assertEqual(req.full_url, ENDPOINT)
        self.assertEqual(options, {"timeout": 20.0})
        self.assertEqual(payload["model"], DEFAULT_MODEL)
        self.assertEqual(payload["max_tokens"], 512)
        self.assertEqual(payload["temperature"], 0)
        self.assertEqual(payload["chat_template_kwargs"], {"enable_thinking": False})
        self.assertEqual(payload["response_format"], {"type": "json_object"})
        self.assertEqual(json.loads(payload["messages"][1]["content"]), OBSERVATION)
        self.assertEqual(evidence["provider"], "nebius-live")
        self.assertEqual(evidence["request_id"], "req-test")
        self.assertEqual(evidence["served_model"], DEFAULT_MODEL)
        self.assertNotIn("test-secret", json.dumps(evidence))

    def test_credential_model_and_timeout_validation(self):
        with patch.dict("os.environ", {}, clear=True):
            with self.assertRaises(ProviderError):
                NebiusProvider()
        for key in ("secret\n", "secret token", "秘密", 123):
            with self.subTest(key=key), self.assertRaises(ProviderError) as caught:
                NebiusProvider(api_key=key)
            self.assertNotIn(str(key), str(caught.exception))
        for model in ("other/model", "nvidia/../../x", "nvidia/", "nvidia/x\n"):
            with self.subTest(model=model), self.assertRaises(ProviderError):
                NebiusProvider(api_key="x", model=model)
        for timeout in (0, -1, 61, float("nan"), float("inf")):
            with self.subTest(timeout=timeout), self.assertRaises(ProviderError):
                NebiusProvider(api_key="x", timeout=timeout)

    def test_rejects_wrong_model_truncation_refusal_tools_and_missing_choices(self):
        malformed = [completion(model="other/model"), completion(choices=[]),
            completion(choices=[{"finish_reason": "length", "message": {"content": "{}"}}]),
            completion(choices=[{"finish_reason": "stop", "message": {"refusal": "No"}}]),
            completion(choices=[{"finish_reason": "stop", "message": {"content": "{}", "tool_calls": [{}]}}]),
            completion(choices=[{"finish_reason": "stop", "message": {"content": None}}])]
        for value in malformed:
            with self.subTest(value=value), self.assertRaises(ProviderError):
                self.provider(value).decide(OBSERVATION)

    def test_strict_decision_shape_and_reason(self):
        for value in ({**DECISION, "target": [1, 2, 3]}, {**DECISION, "action": "execute"},
                      {**DECISION, "action": []}, {**DECISION, "target_id": "other"},
                      {**DECISION, "reason": ""}, {**DECISION, "reason": "bad\nline"},
                      {**DECISION, "reason": "x" * 257}, None):
            with self.subTest(value=value), self.assertRaises(ProviderError):
                validate_decision(value)

    def test_duplicate_output_keys_and_nonfinite_json_rejected(self):
        for content in ('{"action":"hold","action":"reach","target_id":"workpiece","reason":"x"}',
                        '{"action":"hold","target_id":"workpiece","reason":NaN}', "```json\n{}\n```"):
            value = completion(choices=[{"finish_reason": "stop", "message": {"content": content}}])
            with self.subTest(content=content), self.assertRaises(ProviderError):
                self.provider(value).decide(OBSERVATION)

    def test_transport_failure_no_retry_or_body_disclosure(self):
        for error in (HTTPError(ENDPOINT, 401, "test-secret", {}, io.BytesIO(b"test-secret")),
                      URLError("test-secret"), TimeoutError("test-secret")):
            provider = self.provider()
            with patch.object(provider, "_open", side_effect=error) as opened:
                with self.assertRaises(ProviderError) as caught:
                    provider.decide(OBSERVATION)
                self.assertEqual(opened.call_count, 1)
                self.assertNotIn("test-secret", str(caught.exception))

    def test_response_bytes_bounded_and_bad_utf8_rejected(self):
        for value in (b"x" * (MAX_RESPONSE_BYTES + 1), b"\xff", b"[]", b'{"model":NaN}'):
            provider = self.provider()
            provider._open = lambda *a, **k: Response(value)
            with self.subTest(value=value[:20]), self.assertRaises(ProviderError):
                provider.decide(OBSERVATION)

    def test_rejects_nonfinite_or_oversized_observation_before_network(self):
        for obs in ({"x": float("nan")}, {"x": "x" * 40_000}, []):
            provider = self.provider()
            with self.subTest(obs=str(obs)[:40]), self.assertRaises(ProviderError):
                provider.decide(obs)
            self.assertEqual(self.calls, [])

    def test_fixture_label_and_event_choices(self):
        for event, expected in (("initial", "reach"), ("target_moved", "reobserve"),
                                ("stale", "hold"), ("out_of_bounds", "hold")):
            decision, evidence = FixtureProvider().decide({**OBSERVATION, "event": event})
            self.assertEqual(decision["action"], expected)
            self.assertEqual(evidence["provider"], "fixture")
            self.assertIsNone(evidence["served_model"])
        decision, _ = FixtureProvider().decide({"event": "target_moved", "retries_remaining": 0})
        self.assertEqual(decision["action"], "hold")


if __name__ == "__main__":
    unittest.main()
