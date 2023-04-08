designFile = "D:/GitHub/custom-escooter/BLDC Controller PCB/BLDC Motor Controller/PDNAnalyzer_Output/BLDC Motor Controller/odb.tgz"

powerNets = ["36V", "PHASE_A"]

groundNets = ["GND", "PHASE_B"]

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
"power_pins": [ ("J4", "pdna_pin_1_1"), ("J4", "pdna_pin_1_2"), ("J4", "pdna_pin_1_3"), ("J4", "pdna_pin_1_4") ],
"ground_pins": [ ("J5", "pdna_pin_1_1"), ("J5", "pdna_pin_1_2"), ("J5", "pdna_pin_1_3"), ("J5", "pdna_pin_1_4") ],
"current": 20,
"Rpin": 0.02,
}
]


voltage_regulators = [
{
"id": "2",
"type": "linear",

"in": [ ("Q1", "8"), ("Q1", "7"), ("Q1", "6"), ("Q1", "5") ],
"out": [ ("Q1", "1"), ("Q1", "2"), ("Q1", "3") ],
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

"in": [ ("Q4", "1"), ("Q4", "2"), ("Q4", "3") ],
"out": [ ("Q4", "8"), ("Q4", "7"), ("Q4", "6"), ("Q4", "5") ],
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
