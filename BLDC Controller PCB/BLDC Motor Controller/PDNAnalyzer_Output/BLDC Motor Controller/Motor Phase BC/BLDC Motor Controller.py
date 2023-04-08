designFile = "D:/GitHub/custom-escooter/BLDC Controller PCB/BLDC Motor Controller/PDNAnalyzer_Output/BLDC Motor Controller/odb.tgz"

powerNets = ["36V", "PHASE_B"]

groundNets = ["GND", "PHASE_C"]

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
"power_pins": [ ("J5", "pdna_pin_1_1"), ("J5", "pdna_pin_1_2"), ("J5", "pdna_pin_1_3"), ("J5", "pdna_pin_1_4") ],
"ground_pins": [ ("J6", "pdna_pin_1_1"), ("J6", "pdna_pin_1_2"), ("J6", "pdna_pin_1_3"), ("J6", "pdna_pin_1_4") ],
"current": 20,
"Rpin": 0.02,
}
]


voltage_regulators = [
{
"id": "2",
"type": "linear",

"in": [ ("Q6", "1"), ("Q6", "2"), ("Q6", "3") ],
"out": [ ("Q6", "8"), ("Q6", "7"), ("Q6", "6"), ("Q6", "5") ],
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

"in": [ ("Q3", "8"), ("Q3", "7"), ("Q3", "6"), ("Q3", "5") ],
"out": [ ("Q3", "1"), ("Q3", "2"), ("Q3", "3") ],
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
