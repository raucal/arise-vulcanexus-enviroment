import snap7

# # 1. PLC ennection definition  
# IP = '192.168.1.3' # IP del PLC
# RACK = 0
# SLOT = 1

# # Numero de la DB
# DB_NUMBER = 9
# START_ADDRESS = 0    # Inicio 
# SIZE = 8   # Tamaño   

# # 1.1 PLC connection via snap7 client module
# plc = snap7.client.Client()
# plc.connect(IP,RACK,SLOT)

def read_bool(db,byte,bit):
    # 2. Point DB and read data
    # db = plc.db_read(DB_NUMBER,START_ADDRESS,SIZE)
    # 3. Get the bit assigned from the main program
    status = snap7.util.get_bool(db,byte,bit)

    return status 

def read_usint(db,byte):
    # 2. Point DB and read data
    # db = plc.db_read(DB_NUMBER,START_ADDRESS,SIZE)
    # 3. Get the usint assigned from the main program
    status = snap7.util.get_usint(db,byte)
    return status 

def read_real(db,byte):
    # 2. Point DB and read data
    # db = plc.db_read(DB_NUMBER,START_ADDRESS,SIZE)
    # 3. Get the real assigned from the main program
    status = snap7.util.get_real(db,byte)
    return status

def read_int(db,byte):
    # 2. Point DB and read data
    # db = plc.db_read(DB_NUMBER,START_ADDRESS,SIZE)
    # 3. Get the int assigned from the main program
    status = snap7.util.get_int(db,byte)
    return status

def write_bool(plc,db_number,buffer,byte,bit,bitvalue):
    # db = plc.db_read(DB_NUMBER,START_ADDRESS,SIZE)
    snap7.util.set_bool(buffer,byte,bit,bitvalue)
    plc.db_write(db_number,byte,buffer)
    return buffer

def write_int(plc,db_number,buffer,byte,int_value):
    snap7.util.set_int(buffer,byte,int_value)
    plc.db_write(db_number,0,buffer)

def write_real(plc,db_number,buffer,byte,real_value):
    snap7.util.set_real(buffer,byte,real_value)
    plc.db_write(db_number,0,buffer)