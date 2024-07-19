import yaml, datetime, types, os

class cFileGenerator:

    def __init__(self, filePath: str) -> None:
        """
        fileName: Directory and name with extension of the file to generate, no spaces allowed
        """

        self.fullFilePath = filePath
        self.fileName = filePath.split("/")[-1]

        self.file = f"""
//////////////////// AUTO GENERATED FILE ////////////////////

// do not manually modify this file, modify "device_descriptor.yaml" instead
// run autogen.py to update (called at each build by default)

// last updated at {datetime.datetime.now()}

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __{self.fileName.upper().split(".")[0]}_{self.fileName.upper().split(".")[1]}
#define __{self.fileName.upper().split(".")[0]}_{self.fileName.upper().split(".")[1]}

"""

    def define(self, name: str, value: str, comment: str = "") -> None:
        if(comment != ""):
            self.file += f"\n#define {name} {value}\t\t// {comment}"
        else:
            self.file += f"\n#define {name} {value}"

    def include(self, value: str, comment: str = "") -> None:
        if(comment != ""):
            self.file += f"\n#include{value}\t\t// {comment}"
        else:
            self.file += f"\n#include{value}"

    def blankLine(self, count=1, comment: str = "") -> None:
        for i in range(count):
            if(comment != ""):
                self.file += f"\n// {comment}"
            else:
                self.file += f"\n"

    def enum(self, name: str, enums: dict, type: str = "", comment: str = "") -> None:

        self.file += f"\nenum {name}"
        if(type != ""):
            self.file += f": {type}"
        self.file += "{"
        if(comment != ""):
            self.file += f"\t\t// {comment}"
        try:
            self.file += f"\n\t{name}_start = {enums['start']},"
            enums.pop("start")
        except:
            pass
        end = None
        try:
            end = f"\n\t{name}_end = {enums['end']},"
            enums.pop("end")
        except:
            pass

        for enum_index, enum_data in enums.items():
            self.file += f"\n\t{enum_data['name']} = {enum_index},"
            if(enum_data["comment"] != ""):
                self.file += f"\t\t// {enum_data['comment']}"

        if(end is not None):
            self.file += end


        self.file += "\n};"

    def array(self, type: str, name: str, size: int, initValue = None, comment: str = "") -> None:
        self.file += f"\n{type} {name}[{size}]"
        if(isinstance(initValue, int) or isinstance(initValue, float)):
            self.file += f" = {{{str(initValue)*size}}}"
        elif(isinstance(initValue, list)):
            self.file += f" = {{{str(initValue)[1:-1]}}}"
        
        self.file += ";"
        if(comment != ""):
            self.file += f"\t\t// {comment}"

    def ifDefine(self, defineToCheck: str) -> None:
        self.file += f"\n#ifndef {defineToCheck}"

    def ifDefineEnd(self) -> None:
        self.file += f"\n#endif"

    def saveFile(self) -> None:
        self.file += "\n\n#endif"
        f = open(self.fullFilePath, "w")
        f.write(self.file)
        f.close()



currentDir = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
# load descriptor file
try:
    deviceDescriptor = yaml.safe_load(open(currentDir + "/device_descriptor.yaml"))
except yaml.parser.ParserError as e:
    print(f"AUTOGEN: Error while parsing device_descriptor file: {e}")
except FileNotFoundError:
    print("AUTOGEN: device_descriptor.yaml not found")


# generate header file for device firmware
fileGen = cFileGenerator(currentDir + "/Core/Inc/device_descriptor.h")

fileGen.define("HARDWARE_TYPE", deviceDescriptor["hardware"]["type"])
fileGen.define("HARDWARE_VERSION", deviceDescriptor["hardware"]["version"])
fileGen.define("FIRMWARE_VERSION", deviceDescriptor["firmware"]["version"])
fileGen.blankLine(2)

# define hardware params
for param in deviceDescriptor["hard_parameters"].items():
    singleLineDescription = param[1]['description'].replace('\n', ' ')
    if(param[1]['unit'] != ""):
        fileGen.define(param[0].upper(), param[1]["value"], f"({param[1]['unit']}) {singleLineDescription}")
    else:
        fileGen.define(param[0].upper(), param[1]["value"], f"{singleLineDescription}")
fileGen.blankLine(2)


# group registers into their respective data types
registers = {
    "int8_t":{"read/write":[], "read":[], "write":[]},
    "int16_t":{"read/write":[], "read":[], "write":[]},
    "int32_t":{"read/write":[], "read":[], "write":[]},
    "uint8_t":{"read/write":[], "read":[], "write":[]},
    "uint16_t":{"read/write":[], "read":[], "write":[]},
    "uint32_t":{"read/write":[], "read":[], "write":[]},
    "float":{"read/write":[], "read":[], "write":[]}
}

# ADDRESS_SPACE = 2047    # the maximum number of each variable type
# fileGen.define("ADDRESS_SPACE", ADDRESS_SPACE, "the maximum number of each variable type")


for register_name, register_info in deviceDescriptor["registers"].items():

    if(register_info["var_type"] not in registers.keys()):      # catch bad variable types
        raise Exception(f"Unknown var_type: {register_info['var_type']} for {register_name} register")
    
    if(register_info["permissions"].count("read/write") != 0):
        registers[register_info["var_type"]]["read/write"].append([register_name, register_info])
    elif(register_info["permissions"].count("read") != 0):
        registers[register_info["var_type"]]["read"].append([register_name, register_info])
    elif(register_info["permissions"].count("write") != 0):
        registers[register_info["var_type"]]["write"].append([register_name, register_info])
    else:
        raise Exception(f"unknown read/write permissions: ({register_info['permissions']}) for {register_name} register")

# define register sizes
regCounts = {}
for type_name, vars in registers.items():
    regCounts[type_name] = len(vars["read"]) + len(vars["write"]) + len(vars["read/write"])
    fileGen.define(f"{type_name.upper()}_REGISTER_COUNT", regCounts[type_name])
fileGen.blankLine(1)

#define register offsets
offset = 0
for type_name, count in regCounts.items():
    fileGen.define(f"{type_name.upper()}_REGISTER_OFFSET", str(offset))
    offset += int(count)
fileGen.blankLine(1)


# define register arrays and set default values
for type_name, vars in registers.items():
    defaultValues = []
    for permission_type in vars.values():
        for register in permission_type:
            defaultValues.append(register[1]["value"])
    if(regCounts[type_name] > 0):
        fileGen.array(f"inline {type_name}", f"{type_name}_data_array", regCounts[type_name], initValue=defaultValues)
    else:   # always make arrays at least 1 long, even if there are no registers of that type
        fileGen.array(f"inline {type_name}", f"{type_name}_data_array", 1, initValue=0)
fileGen.blankLine(2)

# create enums for all registers
fileGen.blankLine(1, "link each register to a global address")
offset = 0
old_offset = offset
enum_data = {}
for type_name, vars in registers.items():

    enum_data["start"] = offset
    enum_data["end"] = offset
    for register_index, register in enumerate(vars["read/write"]):
        singleLineDescription = register[1]["description"].replace('\n', ' ')
        if(register[1]['unit'] != ""):
            enum_data[str(offset)] = {
                "name":register[0],
                "comment":f"({register[1]['unit']}) {singleLineDescription}"}
        else:
            enum_data[str(offset)] = {
                "name":register[0],
                "comment":f"{singleLineDescription}"}
        enum_data["end"] = offset
        offset += 1
    if(offset == old_offset):
        offset += 1
    old_offset = offset
    fileGen.enum(f"{type_name}_register_rw", enum_data, "uint16_t", f"read/write {type_name} registers")

    enum_data = {}
    #offset += 1
    enum_data["start"] = offset
    enum_data["end"] = offset
    for register_index, register in enumerate(vars["read"]):
        singleLineDescription = register[1]["description"].replace('\n', ' ')
        enum_data[str(offset)] = {
            "name":register[0],
            "comment":f"({register[1]['unit']}) {singleLineDescription}"}
        enum_data["end"] = offset
        offset += 1
    if(offset == old_offset):
        offset += 1
    old_offset = offset
    fileGen.enum(f"{type_name}_register_r", enum_data, "uint16_t", f"read {type_name} registers")

    enum_data = {}
    #offset += 1
    enum_data["start"] = offset
    enum_data["end"] = offset
    for register_index, register in enumerate(vars["write"]):
        singleLineDescription = register[1]["description"].replace('\n', ' ')
        enum_data[str(offset)] = {
            "name":register[0],
            "comment":f"({register[1]['unit']}) {singleLineDescription}"}
        enum_data["end"] = offset
        offset += 1
    if(offset == old_offset):
        offset += 1
    old_offset = offset
    fileGen.enum(f"{type_name}_register_w", enum_data, "uint16_t", f"write {type_name} registers")

    enum_data = {}
    #offset += 1
    fileGen.blankLine(1)
    


fileGen.saveFile()

print("\n\nAUTOGEN: Successfully updated device_descriptor.h\n\n")


