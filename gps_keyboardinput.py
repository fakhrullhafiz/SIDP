def reverse_geocode(lat, lng):
    try:
        url = (
            f"https://maps.googleapis.com/maps/api/geocode/json?"
            f"latlng={lat},{lng}&key={API_KEY}"
        )
        response = requests.get(url)
        data = response.json()

        if data["status"] != "OK":
            return "Unable to determine address"

        results = data["results"]

        # 1) Try to get exact street (best result)
        for r in results:
            if "street_address" in r["types"]:
                return r["formatted_address"]

        # 2) Try to get route (road)
        for r in results:
            if "route" in r["types"]:
                return r["formatted_address"]

        # 3) Try to get a building, shop, or place
        for r in results:
            if (
                "premise" in r["types"] or
                "establishment" in r["types"] or
                "point_of_interest" in r["types"]
            ):
                return r["formatted_address"]

        # 4) If still nothing useful → return second-best (avoid Plus Code)
        for r in results:
            if "plus_code" not in r["types"]:
                return r["formatted_address"]

        # 5) Last resort
        return results[0]["formatted_address"]

    except Exception as e:
        return f"Reverse geocoding failed: {e}"
