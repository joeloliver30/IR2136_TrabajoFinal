import asyncio
import threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import rclpy
from battery_gps_node import BatteryGPSNode
import uvicorn
from std_msgs.msg import UInt16
from geometry_msgs.msg import Pose, PoseArray

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# Arrancamos ROS2 y el nodo en un hilo separado
rclpy.init()
node = BatteryGPSNode()
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

# Gestión de clientes WebSocket
ws_clients = set()

@app.post("/api/command")
async def command(cmd: dict):
    c = cmd.get("command")
    p = cmd.get("payload", {})
    if c == "land":
        node.land_vehicle()
    elif c == "chargeBattery":
        # Simula recarga al 100%
        await node.battery_publisher.publish(UInt16(data=100))
    elif c == "manual":
        node.connection.mav.manual_control_send(
            node.connection.target_system,
            node.connection.target_component,
            p.get("direction", 0), 0, 0, 0, 0
        )
    elif c == "route":
        arr = PoseArray()
        for seg in p.get("text", "").split(";"):
            lat, lon, alt = map(float, seg.split(","))
            pose = Pose()
            pose.position.x = lat
            pose.position.y = lon
            pose.position.z = alt
            arr.poses.append(pose)
        node.trajectory_callback(arr)
    elif c == "aiSuggestRoute":
        # placeholder
        pass
    return {"ok": True}

@app.websocket("/telemetry")
async def telemetry_ws(ws: WebSocket):
    await ws.accept()
    ws_clients.add(ws)
    try:
        while True:
            # Aquí podrías esperar notificaciones del nodo, pero por simplicidad:
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        ws_clients.remove(ws)

# Para que tu nodo pueda enviar telemetría y logs:
def broadcast_all(message: dict):
    for ws in list(ws_clients):
        asyncio.create_task(ws.send_json(message))

# Sobrescribe métodos del nodo para que usen broadcast_all()
orig_publish_battery = node.publish_battery
def patched_publish_battery():
    orig_publish_battery()
    # lee último valor y lo envía
    val = node.last_battery_value
    broadcast_all({"type": "battery", "value": val})
node.publish_battery = patched_publish_battery

orig_trajectory_cb = node.trajectory_callback
def patched_trajectory_cb(msg):
    broadcast_all({"type": "log", "text": f"📦 Recibida trayectoria ({len(msg.poses)} pts)"})
    orig_trajectory_cb(msg)
node.trajectory_callback = patched_trajectory_cb

# Similarmente parchea land_vehicle y execute_next_point para logs...
orig_land = node.land_vehicle
def patched_land():
    orig_land()
    broadcast_all({"type": "log", "text": "🛬 Aterrizando..."})
node.land_vehicle = patched_land

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
