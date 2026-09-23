import odrive

def print_params(obj, prefix=""):
    """Recursively prints all attributes of an ODrive object."""
    for attr in dir(obj):
        if attr.startswith("_"):  # Skip private/internal attributes
            continue
        try:
            value = getattr(obj, attr)
            if callable(value):  # Skip methods
                continue
            if isinstance(value, (int, float, str, bool, list, tuple, dict, type(None))):  
                print(f"{prefix}{attr}: {value}")
            else:  # Recursively explore sub-objects
                print(f"\n{prefix}{attr}:")
                print_params(value, prefix + "  ")
        except Exception as e:
            print(f"{prefix}{attr}: [Error reading: {e}]")

# Connect to ODrive
print("Connecting to ODrive...")
odrv0 = odrive.find_any()
print("Connected!\n")

# Print all parameters
print_params(odrv0)
