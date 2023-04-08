designFile = "D:/GitHub/custom-escooter/BLDC Controller PCB/BLDC Motor Controller/PDNAnalyzer_Output/BLDC Motor Controller/odb.tgz"

powerNets = ["36V", "PHASE_C"]

groundNets = ["GND", "NetQ2_1", "PHASE_A"]

excitation = [
{
"id": "0",
"type": "source",
"power_pins": [ ("J1", "1") ],
"ground_pins": [ ("J1", "2") ],
"voltage": 36,
"Rpin": 0,
}
,
{
"id": "1",
"type": "load",
"power_pins": [ ("J6", "pdna_pin_1_1"), ("J6", "pdna_pin_1_2"), ("J6", "pdna_pin_1_3"), ("J6", "pdna_pin_1_4") ],
"ground_pins": [ ("J4", "pdna_pin_1_1"), ("J4", "pdna_pin_1_2"), ("J4", "pdna_pin_1_3"), ("J4", "pdna_pin_1_4") ],
"current": 20,
"Rpin": 0.02,
}
]


voltage_regulators = [
{
"id": "2",
"type": "linear",

"in": [ ("Q5", "8"), ("Q5", "7"), ("Q5", "6"), ("Q5", "5") ],
"out": [ ("Q5", "1"), ("Q5", "2"), ("Q5", "3") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.00702857142857143,
}
,
{
"id": "3",
"type": "linear",

"in": [ ("R4", "1") ],
"out": [ ("R4", "2") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.0005,
}
,
{
"id": "4",
"type": "linear",

"in": [ ("Q2", "1"), ("Q2", "2"), ("Q2", "3") ],
"out": [ ("Q2", "8"), ("Q2", "7"), ("Q2", "6"), ("Q2", "5") ],
"ref": [],

"v2": 0,
"i1": 0,
"Ro": 1E-06,
"Rpin": 0.00702857142857143,
}
]


# Resistors / Inductors

passives = []


# Material Properties:

tech = [

        {'name': 'TOP_SOLDER', 'DielectricConstant': 3.8, 'Thickness': 1.27E-05},
        {'name': 'L1_-_PRIMARY', 'Conductivity': 47000000, 'Thickness': 7E-05},
        {'name': 'SUBSTRATE-1', 'DielectricConstant': 4.6, 'Thickness': 0.0002},
        {'name': 'L2_-_GND', 'Conductivity': 47000000, 'Thickness': 3.5E-05},
        {'name': 'SUBSTRATE-2', 'DielectricConstant': 4.5, 'Thickness': 0.001065},
        {'name': 'L3_-_GND', 'Conductivity': 47000000, 'Thickness': 3.5E-05},
        {'name': 'SUBSTRATE-3', 'DielectricConstant': 4.6, 'Thickness': 0.0002},
        {'name': 'L4_-_SECONDARY', 'Conductivity': 47000000, 'Thickness': 7E-05},
        {'name': 'BOTTOM_SOLDER', 'DielectricConstant': 3.8, 'Thickness': 1.27E-05}

       ]

special_settings = {'removecutoutsize' : 7.8 }


plating_thickness = 0.7
finished_hole_diameters = False
