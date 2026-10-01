import sys

def simplify_path(path_str):
    path = list(path_str)
    plen = len(path)

    rules = {
        ('L', 'B', 'R'): 'B',
        ('L', 'B', 'S'): 'R',
        ('R', 'B', 'L'): 'B',
        ('S', 'B', 'L'): 'R',
        ('S', 'B', 'S'): 'B',
        ('L', 'B', 'L'): 'S',
        ('R', 'B', 'R'): 'B',
        ('S', 'B', 'R'): 'L',
        ('R', 'B', 'S'): 'L',
    }

    # In online processing (as each turn is added):
    # Whenever a new character is added to path, if plen >= 3 and path[plen-2] == 'B', we reduce and check again recursively.
    # Let's simulate online adding character by character:
    online_path = []
    for char in path_str:
        online_path.append(char)
        while len(online_path) >= 3 and online_path[-2] == 'B':
            sub = (online_path[-3], online_path[-2], online_path[-1])
            if sub in rules:
                r = rules[sub]
                online_path.pop()
                online_path.pop()
                online_path.pop()
                online_path.append(r)
            else:
                break

    return "".join(online_path)

def test_simplify():
    test_cases = [
        ("LBL", "S"),
        ("LBS", "R"),
        ("SBL", "R"),
        ("SBS", "B"),
        ("LBR", "B"),
        ("RBL", "B"),
        ("RBR", "B"),
        ("SBR", "L"),
        ("RBS", "L"),
        ("LBLLBS", "SR"),
        ("LBSLBS", "RR"),
        ("LBLLBS", "SR"),
    ]
    all_pass = True
    for inp, expected in test_cases:
        res = simplify_path(inp)
        if res == expected:
            print(f"PASS: {inp} -> {res}")
        else:
            print(f"FAIL: {inp} -> got {res}, expected {expected}")
            all_pass = False
    return all_pass

def test_speed_scaling():
    search_speed = 150
    fast_speed = 210
    base_advance_ms = 180

    scaled_advance = int(base_advance_ms * (search_speed / float(fast_speed)))
    print(f"Base advance: {base_advance_ms}ms at speed {search_speed} -> Scaled advance: {scaled_advance}ms at speed {fast_speed}")
    assert scaled_advance < base_advance_ms
    print("PASS: Speed scaling math verified.")
    return True

if __name__ == "__main__":
    p1 = test_simplify()
    p2 = test_speed_scaling()
    if p1 and p2:
        print("ALL TESTS PASSED")
        sys.exit(0)
    else:
        print("SOME TESTS FAILED")
        sys.exit(1)
