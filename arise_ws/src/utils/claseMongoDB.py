import pymongo
from rich import print
from bson import ObjectId

class MongoDB():
    def __init__(self, uri, database_name, collection_name, object_id):
        self.uri = uri
        self.database_name = database_name
        self.collection_name = collection_name
        self.object_id = object_id

        self.client, self.mongo_connected = self.mongo_connection()

        if self.mongo_connected:
            self.db, db_ok = self.get_database()
            if db_ok:
                print("[green]<claseMongoDB>Base de datos cargada correctamente")
            else:
                print("[red]<claseMongoDB>Base de datos no cargada")
        else:
            print("[red]<claseMongoDB>Conexión no establecida con base de datos")
        
    def mongo_connection(self):
        try:
            client = pymongo.MongoClient(self.uri)
            print("[green]<claseMongoDB> Conexión establecida con base de datos")
            return client, True
        except Exception as e:
            print(f"[red]<claseMongoDB>/mongo_connection/[ERROR] -> {e}")
            print("[red]<claseMongoDB> Error al cargar la base de datos")
            return None, False

    def get_database(self):
        try:
            return  self.client[self.database_name], True
        except Exception as e:
            print(f"[red]<claseMongoDB>/get_database/[ERROR] -> {e}")
            return e, False

    def get_config(self):
        try:
            collection = self.db[self.collection_name]
            # Buscar el documento donde _id es el ObjectId dado
            query = {"_id": ObjectId(self.object_id)}
            return collection.find_one(query)
        except Exception as e:
            print(f"[red]<claseMongoDB>/get_config/[ERROR] -> {e}")
            return None

    

