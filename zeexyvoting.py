def voting(slot_distances: list[list[int]]) -> list[str]:
    """Voting using both reflection (lights on) and ambient (lights off)
    Highest reflection is white and lowest ambient is black.

    Args:
        slot_distances (list[list[int]]): List of distances for the slots

    Returns:
        list[str]: The result after scanning each slot
    """
    ref_averages = [0] * 8
    amb_averages = [0] * 8

    for i in range(8):
        print("Start slot", i)

        # Drive to the start of the slot (accounting for window)
        while db.distance() < slot_distances[i][0] + 10:
            db.drive(200, 0)
        print("    Start of window")

        # Collect reflection values until the middle of the slot
        reading_count = 0
        tally = 0
        while db.distance() < (slot_distances[i][0] + slot_distances[i][1]) / 2:
            ref_value = left_sensor.reflection()
            tally += ref_value
            reading_count += 1
            db.drive(200, 0)
        ref_averages[i] = tally / reading_count
        print("    Halfway point")

        # Collect ambient values until the end of the window
        reading_count = 0
        tally = 0
        while db.distance() < slot_distances[i][1] - 10:
            amb_value = left_sensor.ambient()
            tally += amb_value
            reading_count += 1
            db.drive(200, 0)
        amb_averages[i] = tally / reading_count
        print("    End of window")

    db.straight(0)
    print(ref_averages, amb_averages)
    colors = ["NONE"] * 8
    south_ref = ref_averages[:4]
    north_ref = ref_averages[4:]
    south_amb = amb_averages[:4]
    north_amb = amb_averages[4:]
    colors[max((v, i) for i, v in enumerate(south_ref))[1]] = "WHITE"
    colors[min((v, i) for i, v in enumerate(south_amb))[1]] = "BLACK"
    colors[max((v, i) for i, v in enumerate(north_ref))[1] + 4] = "WHITE"
    colors[min((v, i) for i, v in enumerate(north_amb))[1] + 4] = "BLACK"

    try:
        buf = bytearray(16 * 2)  # 2 bytes for each unsigned short
        ref = ref_averages + amb_averages
        i = 0
        for d in ref:
            pack_into("H", buf, i, d)
            i += 2
        hub.system.storage(0, write=buf)
    except Exception:
        pass

    return colors
