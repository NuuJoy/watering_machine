import json

generalsetup = {'machineName':'wm_ros',
                'overseer_delay':0.005}
gpiosetup    = [11,13,15]
pwmsetup     = [['baseazim',0,205,410,-90.0,90.0],
                ['basealti',0,205,410,-90.0,90.0],
                ['midnode',0,205,410,-90.0,90.0],
                ['endnode',0,205,410,-90.0,90.0]]
envsensetup  = [['humid_left' , 'DHT11', [17]],
                ['humid_mid'  , 'DHT11', [27]],
                ['humid_right', 'DHT11', [22]],
                ['illum_left' , 'BH1750', [0,False]],
                ['illum_mid'  , 'BH1750', [0,False]],
                ['illum_right', 'BH1750', [0,False]]]
managersetup = []

with open('wm_setupFile.json','w') as outputFile:
    outputFile.write(json.dumps([generalsetup,gpiosetup,pwmsetup,envsensetup,managersetup]))
