import fastapi
import sqlite3
import uuid
import uvicorn
from pydantic import BaseModel
from typing import List
from fastapi.responses import FileResponse

usersDb = "dashboard/users.db"
entrysDb = "dashboard/entries.db"
APIkeys = {}
app = fastapi.FastAPI()
app.title = "Dashboard API"

class Movement(BaseModel):
    time: int  # Unix timestamp
    form: bool  #Boolean true = in, false = out


#db funcs

#write movements to db, with unix time and whether in or out
def write_movements(movements, api_key):
        conn = sqlite3.connect(entrysDb)
        cursor = conn.cursor()
        
        # Create table if it doesn't exist
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS movements (
            id INTEGER PRIMARY KEY,
            timestamp INTEGER,
            is_entry INTEGER,
            apikey TEXT
        )
        ''')
        
        # Insert movements into the database
        for movement in movements:
            cursor.execute(
                "INSERT INTO movements (timestamp, is_entry, apikey) VALUES (?, ?, ?)",
                (movement.time, 1 if movement.form else 0, api_key)
            )
        
        # Commit changes and close connection
        conn.commit()
        conn.close()

#create api key and add to users db
def create_apikey():
    conn = sqlite3.connect(usersDb)
    cursor = conn.cursor()
    
    #create table if it doesn't exist
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY,
        apikey TEXT UNIQUE
    )
    ''')
    
    #generate api key using uuid
    api_key = str(uuid.uuid4())
    
    try:
        #insert new user into the database
        cursor.execute(
            "INSERT INTO users (apikey) VALUES (?)",
            (api_key,)
        )
        conn.commit()
    except sqlite3.IntegrityError:
        api_key = None
    
    conn.close()
    return api_key


#load all api keys from db into memory
def load_apikeys():
    global APIkeys
    conn = sqlite3.connect(usersDb)
    cursor = conn.cursor()
    
    #create table if it doesn't exist
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS users (
        id INTEGER PRIMARY KEY,
        apikey TEXT UNIQUE
    )
    ''')
    print("Loading API keys from database...")
    cursor.execute("SELECT apikey FROM users")
    rows = cursor.fetchall()
    
    APIkeys = {row[0]: True for row in rows}
    print(f"Loaded {len(APIkeys)} API keys.")
    conn.close()

#select from movements db where it takes unix time given and counts the number of entries and exits since then
def get_movements(since, count, api_key):
    conn = sqlite3.connect(entrysDb)
    cursor = conn.cursor()
    
    # Create table if it doesn't exist
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS movements (
        id INTEGER PRIMARY KEY,
        timestamp INTEGER,
        is_entry INTEGER,
        apikey TEXT
    )
    ''')
    
    cursor.execute(
        """
            SELECT "timestamp", is_entry
            FROM movements
            WHERE "timestamp" <= ? AND apikey = ?
            ORDER BY "timestamp" DESC
            LIMIT ?;
        """,        
        (since, api_key, count)
    )
    rows = cursor.fetchall()
    conn.close()
    return [{"timestamp": row[0], "is_entry": bool(row[1])} for row in rows]
            






@app.get("/")
def read_root():
    return {"Hello": "World"}


@app.get("/getMovements/")
def read_movements(date: int, count: int ,api_key: str):
    try:
        if api_key in APIkeys:
            movements = get_movements(date, count, api_key)
            return {"movements": movements}
        else:
            raise fastapi.HTTPException(
                status_code=401,
                detail="Invalid API key"
            )
    except Exception as e:
        print(f"Error retrieving movements: {str(e)}")
        raise fastapi.HTTPException(
            status_code=500,
            detail=f"Failed to retrieve movements: {str(e)}"
        )



@app.post("/movements/")
def create_movements(movements: List[Movement], api_key: str):
    try:
        if api_key in APIkeys:
            write_movements(movements, api_key)
            return {"message": f"{len(movements)} movements recorded"}
        else:
            raise HTTPException(
                status_code=401,
                detail="Invalid API key",
            )
    except Exception as e:
        print(f"Error recording movement: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to record movement: {str(e)}",
        )


#api request to create a new key
@app.post("/createApiKey/")
def api_create_apikey():
    api_key = create_apikey()
    if api_key:
        #add to api keys
        APIkeys[api_key] = True
        return {"api_key": api_key}
    else:
        return {"error": "Could not create API key"}
    

@app.get("/dashboard")
def get_dashboard():
	    return FileResponse("dashboard/dashboard.html")

@app.on_event("startup")
def startup_event():
    load_apikeys()

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)