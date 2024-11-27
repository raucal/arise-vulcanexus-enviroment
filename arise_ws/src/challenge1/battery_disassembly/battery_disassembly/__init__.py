import json
import sys
import os
user = os.getenv('USER')
utils_path = '/home/'+user+'/arise-vulcanexus-enviroment/arise_ws/src/utils'
sys.path.append(utils_path)
import claseMongoDB, claseRobot, claseVentosa, claseLuces, clasePinza
import transforms as trs

parameters_path = '/home/'+user+'/arise-vulcanexus-enviroment/arise_ws/src/challenge1/battery_disassembly/resource/parameters.json'

def get_mongo_config():
    try:
        with open(parameters_path, 'r') as f:
            mongo_params = json.load(f)

        mongo = claseMongoDB.MongoDB(mongo_params["uri"], mongo_params["database_name"], mongo_params["collection_name"], mongo_params["object_id"],)
        config = mongo.get_config()
        return config
    except Exception as e:
        print(f"[red][ERROR] -> {e}")
        return None
