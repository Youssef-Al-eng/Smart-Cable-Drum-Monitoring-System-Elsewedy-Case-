
import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate("electronic-systems-in-action-firebase-adminsdk-fbsvc-3417217150.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://electronic-systems-in-action-default-rtdb.firebaseio.com/'
})

drum_id = "drum1"
drum_data = {
    "lat": 60.34,
    "lng": 60.78,
    "alt": 100,
    'temperature' : 100,
"wire_history": {
        "1678886400": 120,
        "1678890000": 115,
        "1678893600": 110
      }
}

db.reference('drums').child(drum_id).set(drum_data)
print(f"Data for {drum_id} has been written to the database.")