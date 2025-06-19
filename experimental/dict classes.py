
def dict_2_class_add_to_class(classdest, enum_name, enum_dict):
    reverse_dict = {v: k for k, v in enum_dict.items()}

    class EnumClass:
        pass

    for var_name, data in enum_dict.items():
        vn = var_name.replace(' ','_')
        try:
            dat = int(data)
        except:
            dat = data
        setattr(EnumClass, vn, dat)

    def get(cls, value):
        return reverse_dict.get(value)
    
    setattr(EnumClass, 'get', classmethod(get))
    EnumClass.__name__ = enum_name
    enum_class = EnumClass
    setattr(classdest, enum_name, enum_class)


from unavlib.enums import inav_enums
from unavlib.enums import msp_codes

class EmptyClass():
    pass

randomclass = EmptyClass()
inav_class = EmptyClass()

for attr_name in dir(inav_enums):
    if not attr_name.startswith("__"):
        attr = getattr(inav_enums, attr_name)
        if isinstance(attr, dict): 
            dict_2_class_add_to_class(inav_class, attr_name, attr)
            
setattr(randomclass, "inav", inav_class)
dict_2_class_add_to_class(randomclass, "msp", msp_codes.MSPCodes)

print(randomclass.inav.navArmingBlocker.NAV_ARMING_BLOCKER_NONE)  # Output: 0
print(randomclass.inav.navArmingBlocker.get(0))  # Output: PLATFORM_ROVER
print(randomclass.msp.MSP_WP_GETINFO)
print(randomclass.msp.get(20))
