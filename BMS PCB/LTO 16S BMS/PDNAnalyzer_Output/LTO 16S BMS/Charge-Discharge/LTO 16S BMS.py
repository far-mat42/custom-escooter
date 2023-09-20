designFile = "C:/Farris' Files/School/Side Projects/Electric Scooter/custom-escooter/BMS PCB/LTO 16S BMS/PDNAnalyzer_Output/LTO 16S BMS/odb.tgz"

powerNets = ["VBAT+_F", "VBAT+", "VPACK+", "CPACK+"]

groundNets = ["VBAT-", "VPACK-"]

excitation = [
{
"id": "0",
"type": "source",
"power_pins": [ ("J1", "1") ],
"ground_pins": [ ("J1", "2") ],
"voltage": 36.8,
"Rpin": 0,
}
,
{
"id": "1",
"type": "load",
"power_pins": [ ("J7", "1") ],
"ground_pins": [ ("J7", "2") ],
"current": 20,
"Rpin": 0.005,
}
,
{
"id": "2",
"type": "load",
"power_pins": [ ("J1", "1") ],
"ground_pins": [ ("J1", "2") ],
"current": 10,
"Rpin": 0.01,
}
,
{
"id": "3",
"type": "source",
"power_pins": [ ("J15", "1") ],
"ground_pins": [ ("J15", "2") ],
"voltage": 36.8,
"Rpin": 0,
}
]


voltage_regulators = [
{
"id": "4",
"type": "linear",

"in": [ ("J13", "pdna_pin_1_1"), ("J13", "pdna_pin_1_2") ],
"out": [ ("J13", "pdna_pin_2_1"), ("J13", "pdna_pin_2_2") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.0011,
}
,
{
"id": "5",
"type": "linear",

"in": [ ("Q18", "8"), ("Q18", "7"), ("Q18", "6"), ("Q18", "5") ],
"out": [ ("Q18", "2"), ("Q18", "3"), ("Q18", "1") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.00582857142857143,
}
,
{
"id": "6",
"type": "linear",

"in": [ ("Q19", "8"), ("Q19", "7"), ("Q19", "6"), ("Q19", "5") ],
"out": [ ("Q19", "2"), ("Q19", "3"), ("Q19", "1") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.00582857142857143,
}
,
{
"id": "7",
"type": "linear",

"in": [ ("R63", "1") ],
"out": [ ("R63", "2") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.0005,
}
,
{
"id": "8",
"type": "linear",

"in": [ ("R63", "2") ],
"out": [ ("R63", "1") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.0005,
}
,
{
"id": "9",
"type": "linear",

"in": [ ("Q20", "5"), ("Q20", "6"), ("Q20", "7"), ("Q20", "8") ],
"out": [ ("Q20", "1"), ("Q20", "2"), ("Q20", "3") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.00582857142857143,
}
,
{
"id": "10",
"type": "linear",

"in": [ ("J13", "pdna_pin_2_1"), ("J13", "pdna_pin_2_2") ],
"out": [ ("J13", "pdna_pin_1_1"), ("J13", "pdna_pin_1_2") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.0011,
}
]


# Resistors / Inductors

passives = []


# Material Properties:

tech = [

        {'name': 'TOP_SOLDER', 'DielectricConstant': 3.8, 'Thickness': 1.27E-05},
        {'name': 'L1_-_PRIMARY', 'Conductivity': 47000000, 'Thickness': 7.5E-05},
        {'name': 'SUBSTRATE-1', 'DielectricConstant': 4.6, 'Thickness': 0.0002},
        {'name': 'L2_-_GND', 'Conductivity': 47000000, 'Thickness': 3.5E-05},
        {'name': 'SUBSTRATE-2', 'DielectricConstant': 4.5, 'Thickness': 0.001065},
        {'name': 'L3_-_GND', 'Conductivity': 47000000, 'Thickness': 3.5E-05},
        {'name': 'SUBSTRATE-3', 'DielectricConstant': 4.6, 'Thickness': 0.0002},
        {'name': 'L4_-_SECONDARY', 'Conductivity': 47000000, 'Thickness': 7.5E-05},
        {'name': 'BOTTOM_SOLDER', 'DielectricConstant': 3.8, 'Thickness': 1.27E-05}

       ]

special_settings = {'removecutoutsize' : 7.8 }


plating_thickness = 0.7
finished_hole_diameters = False
