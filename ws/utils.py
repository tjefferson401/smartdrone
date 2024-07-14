import json
from flask import jsonify, request


def parse_payload(payload): 
    # Parse the payload to json
    try:
        payload = json.loads(payload)
    except json.JSONDecodeError as e:
        return {"error": "Invalid JSON payload"}

    return payload