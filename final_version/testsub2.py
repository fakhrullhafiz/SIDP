#!/usr/bin/env python3
"""
testsub2.py

Utility to produce a friendlier location string for given coordinates.
It prefers a human-readable place name (Places API) over raw plus-codes from Geocoding.

Usage (from the `final_version` folder):
  python testsub2.py --lat 4.000 --lng 101.000

It will print the raw geocode formatted_address and the improved friendly
string that you can use in TTS instead of the raw plus code.
"""

import argparse
import os
import re
import requests
from typing import Optional, Tuple

# Try to reuse API_KEY from existing module if available
try:
    from sub2 import API_KEY
except Exception:
    API_KEY = os.environ.get("GOOGLE_API_KEY")


def geocode_latlng(lat: float, lng: float, api_key: str):
    url = (
        f"https://maps.googleapis.com/maps/api/geocode/json?latlng={lat},{lng}&key={api_key}"
    )
    r = requests.get(url, timeout=6)
    return r.json()


def places_nearby(lat: float, lng: float, api_key: str) -> Tuple[Optional[str], Optional[str]]:
    # Try a nearbysearch ranked by distance; if the API requires a type/keyword,
    # some deployments may need to add &keyword= or &type=; keep as simple attempt.
    url = (
        f"https://maps.googleapis.com/maps/api/place/nearbysearch/json?location={lat},{lng}&rankby=distance&key={api_key}"
    )
    try:
        r = requests.get(url, timeout=6)
        js = r.json()
        if js.get("status") == "OK" and js.get("results"):
            top = js["results"][0]
            name = top.get("name")
            vicinity = top.get("vicinity") or top.get("formatted_address")
            return name, vicinity
    except Exception:
        pass
    return None, None


def nice_address_from_components(components: list) -> Optional[str]:
    # Pick a compact, human-friendly combination of locality/admin/country
    mapping = {t: [] for t in (
        "postal_code",
        "locality",
        "sublocality",
        "administrative_area_level_2",
        "administrative_area_level_1",
        "country",
    )}
    for c in components:
        for t in c.get("types", []):
            if t in mapping:
                mapping[t].append(c.get("long_name"))

    parts = []
    # prefer locality or sublocality
    if mapping["locality"]:
        parts.append(mapping["locality"][0])
    elif mapping["sublocality"]:
        parts.append(mapping["sublocality"][0])

    # administrative area
    if mapping["administrative_area_level_1"]:
        parts.append(mapping["administrative_area_level_1"][0])
    elif mapping["administrative_area_level_2"]:
        parts.append(mapping["administrative_area_level_2"][0])

    # country
    if mapping["country"]:
        parts.append(mapping["country"][0])

    if parts:
        return ", ".join(parts)
    return None


def looks_like_plus_code(formatted_address: Optional[str], geocode_json: dict) -> bool:
    # If the geocode response includes a top-level plus_code compound_code,
    # that's a strong signal it's returning an Open Location Code (plus code).
    if not formatted_address:
        return False
    if geocode_json.get("plus_code") and geocode_json["plus_code"].get("compound_code"):
        return True
    # Heuristic: first token contains a '+' and only a few chars (e.g. '9XP8+RH')
    first = formatted_address.split(",")[0].strip()
    if "+" in first and len(first) <= 12 and re.search(r"[A-Z0-9]+\+[A-Z0-9]+", first):
        return True
    return False


def friendly_location(lat: float, lng: float, api_key: Optional[str] = None) -> str:
    """Return a friendlier location string to use for TTS.

    Strategy:
    - Call Geocoding API for formatted_address and address_components.
    - If the geocoder returns a plus-code-like address, attempt Places Nearby
      to get a human-friendly place name (name + vicinity).
    - If Places fails, construct a short address from address_components.
    - Otherwise return the formatted_address.
    """
    if api_key is None:
        api_key = API_KEY
    if not api_key:
        raise RuntimeError("No Google API key available. Set sub2.API_KEY or GOOGLE_API_KEY env var.")

    js = geocode_latlng(lat, lng, api_key)
    formatted = None
    try:
        if js.get("status") == "OK" and js.get("results"):
            formatted = js["results"][0].get("formatted_address")
    except Exception:
        formatted = None

    if looks_like_plus_code(formatted, js):
        # Try Places for a friendly name first
        name, vicinity = places_nearby(lat, lng, api_key)
        if name:
            if vicinity:
                return f"{name}. {vicinity}"
            return name
        # Fallback: try to make a short nice address from components
        try:
            comp = js.get("results", [])[0].get("address_components", [])
            nice = nice_address_from_components(comp)
            if nice:
                return nice
        except Exception:
            pass

    # If not a plus code or fallbacks failed, return formatted if available
    if formatted:
        return formatted
    return "Unknown location"


def main():
    p = argparse.ArgumentParser(description="Test friendly location output for coordinates")
    p.add_argument("--lat", type=float, help="Latitude", required=False)
    p.add_argument("--lng", type=float, help="Longitude", required=False)
    args = p.parse_args()

    if args.lat is None or args.lng is None:
        try:
            args.lat = float(input("Latitude: "))
            args.lng = float(input("Longitude: "))
        except Exception:
            print("Invalid input; exiting.")
            return

    print("Using API key from sub2.py or GOOGLE_API_KEY env var.")
    try:
        raw = geocode_latlng(args.lat, args.lng, API_KEY)
        raw_addr = None
        if raw.get("status") == "OK" and raw.get("results"):
            raw_addr = raw["results"][0].get("formatted_address")
        print("Raw geocode formatted_address:", raw_addr)
    except Exception as e:
        print("Geocode request failed:", e)

    try:
        friendly = friendly_location(args.lat, args.lng, API_KEY)
        print("Friendly location:", friendly)
    except Exception as e:
        print("friendly_location failed:", e)


if __name__ == "__main__":
    main()
