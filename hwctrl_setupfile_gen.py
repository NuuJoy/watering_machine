import json

generalsetup = {'machineName':'wm_ros'}

gpiosetup = [11,13,15]

pwmsetup  = [['baseazim',0,205,410,-90.0,90.0],['basealti',0,205,410,-90.0,90.0],['midnode',0,205,410,-90.0,90.0],['endnode',0,205,410,-90.0,90.0]]

with open('wm_setupFile.json','w') as outputFile:
    outputFile.write(json.dumps([generalsetup,gpiosetup, pwmsetup]))
