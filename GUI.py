from flask import Flask, jsonify, render_template_string, send_file
import firebase_admin
from firebase_admin import credentials, db
import datetime
import pandas as pd
from io import BytesIO

# Initialize Flask app
app = Flask(__name__, static_url_path='/static', static_folder='static')

# Initialize Firebase using your credentials JSON file
cred = credentials.Certificate("electronic-systems-in-action-firebase-adminsdk-fbsvc-3417217150.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://electronic-systems-in-action-default-rtdb.firebaseio.com/'
})


def predict_cable_end_time(wire_history):
    sorted_list = []
    for k, v in sorted(wire_history.items(), key=lambda x: int(x[0])):
        sorted_list.append([int(k), v])
    sorted_list.sort()
    if len(wire_history) == 0:
        return "no enough data", 6
    elif len(wire_history) == 1:
        return "no enough data", sorted_list[-1][1]
    mx_slope = 0.0
    for i in range(len(sorted_list) - 1):
        if sorted_list[i][1] > sorted_list[i + 1][1]:
            mx_slope = max(mx_slope,
                           (sorted_list[i][1] - sorted_list[i + 1][1]) / (sorted_list[i + 1][0] - sorted_list[i][0]))
    if mx_slope < 0.01:
        return "No change is happened", sorted_list[-1][1]
    nw = sorted_list[-1][0]
    change = sorted_list[-1][1] / mx_slope
    predicted_end_time = datetime.datetime.fromtimestamp(nw + change).strftime('%m/%d/%Y %I:%M:%S %p')
    return predicted_end_time, sorted_list[-1][1]


def get_marker_data():
    drums_ref = db.reference('drums')
    data = drums_ref.get()
    markers = []
    if data:
        for drum_id, drum in data.items():
            wire_history = drum.get("wire_history", {})
            # Format history for display.
            sorted_history = dict(sorted(wire_history.items()))
            formatted_wire_history = {
                datetime.datetime.fromtimestamp(int(ts)).strftime('%m/%d/%Y %I:%M:%S %p'): value
                for ts, value in sorted_history.items()
            }
            finish_time, last_val = predict_cable_end_time(wire_history)
            # If last reading is <= 5 then trigger low cable warning.
            try:
                last_value = float(last_val)
            except (ValueError, TypeError):
                last_value = 0
            low_cable = True if last_value <= 5 else False

            marker = {
                "id": drum_id,
                "lat": drum.get("lat"),
                "lng": drum.get("lng"),
                "wire_history": formatted_wire_history,
                "temperature": drum.get("temperature", "N/A"),
                "predicted_finish_time": finish_time,
                "low_cable": low_cable,
                "last_cable_length": last_val
            }
            markers.append(marker)
    return markers


@app.route("/get_markers")
def markers():
    return jsonify(get_marker_data())


@app.route("/get_marker_data/<drum_id>")
def get_single_marker_data(drum_id):
    drums_ref = db.reference('drums/' + drum_id)
    drum = drums_ref.get()
    if drum:
        wire_history = drum.get("wire_history", {})
        sorted_history = dict(sorted(wire_history.items()))
        formatted_wire_history = {
            datetime.datetime.fromtimestamp(int(ts)).strftime('%m/%d/%Y %I:%M:%S %p'): value
            for ts, value in sorted_history.items()
        }
        finish_time, last_val = predict_cable_end_time(wire_history)
        try:
            last_value = float(last_val)
        except (ValueError, TypeError):
            last_value = 0
        low_cable = True if last_value <= 5 else False

        marker = {
            "id": drum_id,
            "lat": drum.get("lat"),
            "lng": drum.get("lng"),
            "wire_history": formatted_wire_history,
            "temperature": drum.get("temperature", "N/A"),
            "predicted_finish_time": finish_time,
            "low_cable": low_cable,
            "last_cable_length": last_val
        }
        return jsonify(marker)
    else:
        return jsonify({"error": "Drum not found"}), 404


@app.route("/download_summary")
def download_summary():
    drums_ref = db.reference('drums')
    data = drums_ref.get()
    summary_data = []
    if data:
        for drum_id, drum in data.items():
            wire_history = drum.get("wire_history", {})
            finish_time, last_val = predict_cable_end_time(wire_history)
            try:
                last_value = float(last_val)
            except (ValueError, TypeError):
                last_value = 0
            low_cable = True if last_value <= 5 else False

            summary_data.append({
                "Drum ID": drum_id,
                "Latitude": drum.get("lat"),
                "Longitude": drum.get("lng"),
                "Last Cable Length (m)": last_val,
                "Predicted Finish Time": finish_time,
                "Temperature (°C)": drum.get("temperature", "N/A"),
                "Low Cable Warning": "Yes" if low_cable else "No"
            })

    df = pd.DataFrame(summary_data)
    excel_buffer = BytesIO()
    df.to_excel(excel_buffer, index=False, sheet_name='Cable Drum Summary')
    excel_buffer.seek(0)

    return send_file(excel_buffer, download_name='cable_drum_summary.xlsx',
                     mimetype='application/vnd.openxmlformats-officedocument.spreadsheetml.sheet')


@app.route("/")
def index():
    html = """
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset='utf-8'>
      <meta name='viewport' content='width=device-width, initial-scale=1.0'>
      <title>Cable Management System</title>
      <link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css' />
      <style>
        /* Basic resets and layout */
        * { margin: 0; padding: 0; box-sizing: border-box; }
        html, body { width: 100%; height: 100%; overflow: hidden; font-family: Arial, sans-serif; }

        /* Splash Screen - keeping original screen layout but improving responsiveness */
        .splash-screen {
          position: fixed;
          top: 0; left: 0;
          width: 100%; height: 100%;
          background: url('/static/image.jpg') no-repeat center center;
          background-size: cover;
          transition: transform 0.8s cubic-bezier(0.4, 0, 0.2, 1);
          z-index: 1000;
        }
        .splash-screen.slide-out { transform: translateX(-100%); }

        /* Map container */
        .app-content {
          position: fixed;
          top: 0; right: -100%;
          width: 100%; height: 100%;
          transition: transform 0.8s cubic-bezier(0.4, 0, 0.2, 1);
        }
        .app-content.slide-in { transform: translateX(-100%); }
        #map { width: 100%; height: 100%; }

        /* Full-screen Modal for Marker Data */
        .modal-overlay {
          position: fixed;
          top: 0; left: 0;
          width: 100%; height: 100%;
          background: rgba(0, 0, 0, 0.7);
          display: none;
          align-items: center;
          justify-content: center;
          z-index: 2000;
        }
        .modal-overlay.active { display: flex; }
        .modal-content {
          width: 90%;
          max-width: 600px;
          max-height: 90vh;
          overflow-y: auto;
          background: #fff;
          padding: 20px;
          border-radius: 8px;
          position: relative;
        }
        .modal-content img.logo {
          width: 100%;
          height: auto;
          display: block;
          margin-bottom: 10px;
        }
        .modal-content table {
          border-collapse: collapse;
          width: 100%;
          margin-bottom: 10px;
        }
        .modal-content table th, .modal-content table td {
          border: 1px solid blue;
          padding: 8px;
        }
        .modal-close {
          position: absolute;
          top: 10px;
          right: 10px;
          font-size: 1.5em;
          background: transparent;
          border: none;
          cursor: pointer;
        }

        /* Original button style, just adding minor responsive behavior */
        .enter-btn {
          position: absolute;
          bottom: 20px;
          left: 50%;
          transform: translateX(-50%);
          padding: 15px 40px 15px 25px;
          font-size: 1.5em;
          background: rgba(0, 85, 255, 0.9);
          color: white;
          border: none;
          border-radius: 30px;
          cursor: pointer;
          transition: all 0.3s ease;
          backdrop-filter: blur(5px);
          min-width: 300px;
          text-align: left;
        }
        .enter-btn:hover {
          background: rgba(0, 65, 200, 0.9);
          transform: translateX(-50%) scale(1.05);
          padding-right: 50px;
        }
        .enter-btn::after {
          content: '→';
          position: absolute;
          right: 15px;
          top: 50%;
          transform: translateY(-50%);
          transition: all 0.3s ease;
        }

        /* Warning row style */
        .low-cable-warning {
          background-color: #ffcccc;
          font-weight: bold;
          color: red;
        }

        /* Download button style */
        .download-btn {
          position: absolute;
          top: 20px;
          right: 20px;
          padding: 10px 15px;
          font-size: 1em;
          background-color: #4CAF50;
          color: white;
          border: none;
          border-radius: 5px;
          cursor: pointer;
          z-index: 1001; /* Ensure it's above the map */
        }
        .download-btn:hover {
          background-color: #45a049;
        }

        /* Media queries for better mobile responsiveness, but keeping original button size */
        @media screen and (max-width: 768px) {
          .modal-content {
            width: 95%;
            padding: 15px;
          }

          .download-btn {
            top: 10px;
            right: 10px;
            padding: 8px 12px;
            font-size: 0.9em;
          }
        }

        @media screen and (max-width: 480px) {
          .modal-content table th, .modal-content table td {
            padding: 5px;
            font-size: 0.9em;
          }

          /* Make the splash screen fit better */
          .splash-screen {
            background-position: center;
            background-size: cover;
          }
        }
      </style>
    </head>
    <body>
      <div class="splash-screen" id="splashScreen">
        <button class="enter-btn" id="enterBtn">Manage your cable drums</button>
      </div>

      <div class="app-content" id="appContent">
        <button class="download-btn" onclick="downloadSummary()">Download Excel Summary</button>
        <div id="map"></div>
      </div>

      <div class="modal-overlay" id="modalOverlay">
        <div class="modal-content" id="modalContent">
          <button class="modal-close" id="modalClose">&times;</button>
          </div>
      </div>

      <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
      <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
      <script>
        // Global variables for chart instance and refresh control.
        var currentChart = null;
        var modalRefreshInterval = null;
        var currentDrumId = null;

        // Function to trigger the download of the summary.
        function downloadSummary() {
          window.location.href = '/download_summary';
        }

        // Function to populate modal with drum data.
        function populateModal(markerData) {
          currentDrumId = markerData.id; // store current drum id globally
          const tempValue = Number(markerData.temperature);
          const tempStatus = !isNaN(tempValue)
              ? tempValue < 40
                  ? "[Safety mode]"
                  : "<b style='color:red;'>Warning ⚠️</b>"
              : "";

          // Build table with an extra row for Last Cable Reading.
          let tableHTML = `
            <table id="drumDataTable">
              <tr>
                <th>Drum ID</th>
                <td id="drumIdField">${markerData.id}</td>
              </tr>
              <tr>
                <th>Last Cable Reading</th>
                <td id="lastCableField">${markerData.last_cable_length}</td>
              </tr>
              <tr>
                <th>Temperature</th>
                <td id="temperatureField">${markerData.temperature} °C ${tempStatus}</td>
              </tr>
              <tr>
                <th>Predicted Finish Time</th>
                <td id="finishTimeField">${markerData.predicted_finish_time}</td>
              </tr>
              <tr id="warningRow" class="${markerData.low_cable ? 'low-cable-warning' : 'hidden'}" style="${markerData.low_cable ? '' : 'display:none;'}">
                <td colspan="2"><b>LOW CABLE WARNING!</b></td>
              </tr>
            </table>
          `;

          // Build modal content with logo and canvas for chart.
          const contentHTML = `
            <img src="/static/image2.jpg" alt="Logo" class="logo" />
            ${tableHTML}
            <canvas id="chart-${markerData.id}" width="300" height="200"></canvas>
          `;

          // Preserve the close button and then update modal content.
          document.getElementById('modalContent').innerHTML = '<button class="modal-close" id="modalClose">&times;</button>' + contentHTML;
          document.getElementById('modalClose').addEventListener('click', hideModal);

          // Create the chart instance with the current data.
          createChart(`chart-${markerData.id}`, markerData.wire_history);
        }

        // Create chart instance.
        function createChart(chartId, wireHistory) {
          const labels = Object.keys(wireHistory);
          const dataPoints = Object.values(wireHistory).map(Number);
          if (labels.length < 2) return;
          const ctx = document.getElementById(chartId).getContext('2d');
          currentChart = new Chart(ctx, {
            type: 'line',
            data: {
              labels: labels,
              datasets: [{
                label: 'Cable Length (m)',
                data: dataPoints,
                fill: true,
                backgroundColor: 'rgba(54, 162, 235, 0.2)',
                borderColor: 'rgba(54, 162, 235, 1)',
                borderWidth: 4,
                tension: 0.4,
                pointBackgroundColor: '#1e88e5'
              }]
            },
            options: {
              responsive: true,
              plugins: {
                legend: { display: false }
              },
              scales: {
                x: { title: { display: true, text: 'Time' } },
                y: { beginAtZero: true, min: 0, max: 11, title: { display: true, text: 'Cable Length (m)' } }
              }
            }
          });
        }

        // Refresh modal data: update table fields and, if new data exists, append only the new point.
        function refreshModalData() {
          fetch(`/get_marker_data/${currentDrumId}`)
            .then(response => response.json())
            .then(updatedData => {
              const tempVal = Number(updatedData.temperature);
              const tempStatus = !isNaN(tempVal)
                    ? tempVal < 40
                        ? "[Safety mode]"
                        : "<b style='color:red;'>Warning ⚠️</b>"
                    : "";
              document.getElementById('temperatureField').innerHTML = updatedData.temperature + " °C " + tempStatus;
              document.getElementById('finishTimeField').innerHTML = updatedData.predicted_finish_time;
              document.getElementById('lastCableField').innerHTML = updatedData.last_cable_length;

              // Dynamically update the low cable warning row
              const warningRow = document.getElementById('warningRow');
              if (updatedData.low_cable) {
                warningRow.style.display = ''; // Show the warning row
                warningRow.className = 'low-cable-warning';
              } else {
                warningRow.style.display = 'none'; // Hide the warning row
                warningRow.className = 'hidden';
              }

              // Update chart if there is a new data point.
              const newLabels = Object.keys(updatedData.wire_history);
              const newData = Object.values(updatedData.wire_history).map(Number);
              if (currentChart !== null && newLabels.length > currentChart.data.labels.length) {
                for (let i = currentChart.data.labels.length; i < newLabels.length; i++) {
                  currentChart.data.labels.push(newLabels[i]);
                  currentChart.data.datasets[0].data.push(newData[i]);
                }
                currentChart.update();
              }
            })
            .catch(console.error);
        }

        // Show and hide modal functions.
        function showModal(markerData) {
          currentChart = null;
          populateModal(markerData);
          document.getElementById('modalOverlay').classList.add('active');
          modalRefreshInterval = setInterval(refreshModalData, 1000);
        }

        function hideModal() {
          document.getElementById('modalOverlay').classList.remove('active');
          if (modalRefreshInterval) {
            clearInterval(modalRefreshInterval);
            modalRefreshInterval = null;
          }
          if (currentChart !== null) {
            currentChart.destroy();
            currentChart = null;
          }
        }

        // Initialize the Leaflet map and markers.
        function initializeMap() {
          var map = L.map('map').setView([26.8206, 30.8025], 6);
          L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);

          var markersLayer = L.layerGroup().addTo(map);

          function updateMarkers() {
            fetch('/get_markers')
              .then(response => response.json())
              .then(data => {
                markersLayer.clearLayers();
                data.forEach(markerData => {
                  if (markerData.lat && markerData.lng) {
                    var markerOptions = {};
                    if (markerData.low_cable) {
                      markerOptions.icon = L.icon({
                        iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png'
                      });
                    }
                    else if (markerData.temperature && !isNaN(parseFloat(markerData.temperature)) && parseFloat(markerData.temperature) >= 40) {
                      markerOptions.icon = L.icon({
                        iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-yellow.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png'
                      });
                    } 
                    // Else use default blue marker.
                    else {
                      markerOptions.icon = L.icon({
                        iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-blue.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34],
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png'
                      });
                    }
                    L.marker([markerData.lat, markerData.lng], markerOptions)
                      .addTo(markersLayer)
                      .on('click', function() {
                        showModal(markerData);
                      });
                  }
                });
              })
              .catch(console.error);
          }
          setInterval(updateMarkers, 10000);
          updateMarkers();
        }

        document.getElementById('enterBtn').addEventListener('click', function () {
          document.getElementById('splashScreen').classList.add('slide-out');
          document.getElementById('appContent').classList.add('slide-in');
          setTimeout(initializeMap, 500);
        });

        document.getElementById('modalOverlay').addEventListener('click', function(event) {
          if (event.target === this) {
            hideModal();
          }
        });
      </script>
    </body>
    </html>
    """
    return render_template_string(html)


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=5000)