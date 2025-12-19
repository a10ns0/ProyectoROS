# src/truck_positioning/truck_positioning/database_manager.py
from sqlalchemy import create_engine, text

class Database:
    def __init__(self):
        # Tus credenciales
        self.servidor = "192.168.37.12"
        self.usuario = "user_gruas"
        self.clave = "user_gruas"
        self.base_de_datos = "BlackDog"
        self.puerto = 1433
        self.motor = "mssql+pymssql"
        
        self.engine = None
        self.conexion = None
        self.conectar()

    def conectar(self):
        try:
            uri = f"{self.motor}://{self.usuario}:{self.clave}@{self.servidor}:{self.puerto}/{self.base_de_datos}"
            self.engine = create_engine(uri, pool_pre_ping=True)
            self.conexion = self.engine.connect()
            print("DB: Conectado a BlackDog")
            return True
        except Exception as e:
            print(f"DB Error: {e}")
            return False

    def obtener_configuracion_grua(self, id_grua):
        """ Retorna (angulo, limite, estado) o None """
        if not self.conexion:
            if not self.conectar():
                return None
        
        try:
            # AJUSTAR LA CONSULTA SEGÚN TABLA Y CAMPOS REALES
            #query = text(f"SELECT angulo_apertura, distancia_limite, estado FROM ConfiguracionGrua WHERE id = {id_grua}")
            result = self.conexion.execute(query).fetchone()
            return result
        
        except Exception as e:
            print(f"Error Query: {e}")
            self.desconectar()
            return None

    def desconectar(self):
        if self.conexion:
            self.conexion.close()
            self.conexion = None