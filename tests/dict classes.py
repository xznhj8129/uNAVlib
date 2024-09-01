class Enums:
    def __init__(self, dicts):
        for class_name, data in dicts.items():
            enum_class = self._create_enum_class(class_name, data)
            setattr(self, class_name, enum_class)

    def _create_enum_class(self, class_name, data):
        reverse_dict = {v: k for k, v in data.items()}

        class EnumClass:
            pass

        # Set attributes on the EnumClass
        for key, value in data.items():
            setattr(EnumClass, key, value)

        # Add the reverse lookup method
        def get(cls, value):
            return reverse_dict.get(value)
        
        setattr(EnumClass, 'get', classmethod(get))

        EnumClass.__name__ = class_name
        return EnumClass

# Example usage:

# Define your dictionaries
dicts = {
    "PlatformType": {
        "PLATFORM_MULTIROTOR": 0,
        "PLATFORM_AIRPLANE": 1,
        "PLATFORM_HELICOPTER": 2,
        "PLATFORM_TRICOPTER": 3,
        "PLATFORM_ROVER": 4,
        "PLATFORM_BOAT": 5,
        "PLATFORM_OTHER": 6
    },
    "VehicleType": {
        "VEHICLE_CAR": 0,
        "VEHICLE_TRUCK": 1,
        "VEHICLE_MOTORCYCLE": 2,
        "VEHICLE_BICYCLE": 3,
        "VEHICLE_BUS": 4,
        "VEHICLE_TRAIN": 5,
        "VEHICLE_AIRPLANE": 6
    }
}

# Instantiate the Enums class with your dictionaries
enums = Enums(dicts)

# Access the enum values without needing to instantiate the classes
print(enums.PlatformType.PLATFORM_MULTIROTOR)  # Output: 0
print(enums.VehicleType.VEHICLE_TRAIN)  # Output: 5

# Use the reverse lookup
print(enums.PlatformType.get(4))  # Output: PLATFORM_ROVER
print(enums.VehicleType.get(3))  # Output: VEHICLE_BICYCLE
