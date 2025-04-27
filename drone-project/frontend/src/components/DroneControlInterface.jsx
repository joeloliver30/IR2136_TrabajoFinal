import React, { useState, useEffect, useCallback } from "react";
import { Card, CardHeader, CardContent } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Progress } from "@/components/ui/progress";
import { Textarea } from "@/components/ui/textarea";
import { motion } from "framer-motion";
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Move,
  BatteryCharging,
  LandPlot,
  Route,
} from "lucide-react";

async function sendCommand(command, payload = {}) {
  await fetch("http://localhost:8001/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ command, payload }),
  });
}

export default function DroneControlInterface() {
  const [battery, setBattery] = useState(100);
  const [routeText, setRouteText] = useState("");
  const [logs, setLogs] = useState([]);

  // Conectar WebSocket para telemetría y logs
  useEffect(() => {
    const ws = new WebSocket("ws://localhost:8001/telemetry");
    ws.onmessage = (evt) => {
      const msg = JSON.parse(evt.data);
      if (msg.type === "battery") {
        setBattery(msg.value);
      } else if (msg.type === "log") {
        setLogs((prev) => [...prev, msg.text]);
      }
    };
    return () => ws.close();
  }, []);

  // Atajos de teclado W/A/S/D y flechas
  const handleKey = useCallback((e) => {
    const map = {
      w: "forward",
      s: "backward",
      a: "left",
      d: "right",
      ArrowUp: "up",
      ArrowDown: "down",
    };
    if (map[e.key]) {
      sendCommand("manual", { direction: map[e.key] });
    }
  }, []);

  useEffect(() => {
    window.addEventListener("keydown", handleKey);
    return () => window.removeEventListener("keydown", handleKey);
  }, [handleKey]);

  // Handlers de botones
  const handleLand = () => sendCommand("land");
  const handleCharge = () => sendCommand("chargeBattery");
  const handleManual = (direction) => sendCommand("manual", { direction });
  const handleSendRoute = () => {
    if (!routeText.trim()) return;
    sendCommand("route", { text: routeText });
    setRouteText("");
  };

  // Botón de control manual con animaciones
  const ControlButton = ({ icon: Icon, label, onClick }) => (
    <motion.div whileHover={{ scale: 1.05 }} whileTap={{ scale: 0.95 }}>
      <Button
        variant="secondary"
        className="flex flex-col gap-1 p-4 w-24 h-24 text-sm"
        onClick={onClick}
      >
        <Icon className="h-6 w-6" />
        {label}
      </Button>
    </motion.div>
  );

  return (
    <div className="grid grid-cols-1 lg:grid-cols-2 h-screen gap-4 p-4 bg-neutral-900 text-white">
      {/* --- Vista Unity --- */}
      <Card className="w-full h-full bg-neutral-800 border-neutral-700 overflow-hidden">
        <CardHeader>
          <h2 className="text-xl font-semibold">Simulación Unity</h2>
        </CardHeader>
        <CardContent className="p-0 h-full">
          <iframe
            title="Unity Simulation"
            src="http://localhost:8080"
            className="w-full h-full border-none"
            allowFullScreen
          />
        </CardContent>
      </Card>

      {/* --- Panel de Control y Logs --- */}
      <div className="flex flex-col gap-4 overflow-y-auto">
        {/* Batería */}
        <Card className="bg-neutral-800 border-neutral-700">
          <CardHeader className="flex items-center justify-between pb-2">
            <h3 className="text-lg font-medium">Batería</h3>
            <BatteryCharging className="h-5 w-5" />
          </CardHeader>
          <CardContent>
            <Progress value={battery} className="mb-2" />
            <div className="flex justify-between items-center">
              <span>{battery.toFixed(0)}%</span>
              <Button size="sm" onClick={handleCharge}>
                Cargar batería
              </Button>
            </div>
          </CardContent>
        </Card>

        {/* Aterrizar & Ruta */}
        <div className="grid grid-cols-2 gap-4">
          <Card className="bg-neutral-800 border-neutral-700">
            <CardHeader className="flex items-center justify-between pb-2">
              <h3 className="text-lg font-medium">Aterrizaje</h3>
              <LandPlot className="h-5 w-5" />
            </CardHeader>
            <CardContent>
              <Button className="w-full" onClick={handleLand}>
                Descender ahora
              </Button>
            </CardContent>
          </Card>

          <Card className="bg-neutral-800 border-neutral-700">
            <CardHeader className="flex items-center justify-between pb-2">
              <h3 className="text-lg font-medium">Ruta</h3>
              <Route className="h-5 w-5" />
            </CardHeader>
            <CardContent className="space-y-2">
              <Textarea
                placeholder="Lat,Lon,Alt; Lat,Lon,Alt; ..."
                value={routeText}
                onChange={(e) => setRouteText(e.target.value)}
                className="text-black"
                rows={3}
              />
              <Button className="w-full" onClick={handleSendRoute}>
                Enviar ruta
              </Button>
            </CardContent>
          </Card>
        </div>

        {/* Control Manual */}
        <Card className="bg-neutral-800 border-neutral-700">
          <CardHeader className="flex items-center justify-between pb-2">
            <h3 className="text-lg font-medium">Control Manual</h3>
            <Move className="h-5 w-5" />
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-3 gap-2 place-items-center">
              <div />
              <ControlButton
                icon={ArrowUp}
                label="Arriba"
                onClick={() => handleManual("forward")}
              />
              <div />
              <ControlButton
                icon={ArrowLeft}
                label="Izq."
                onClick={() => handleManual("left")}
              />
              <ControlButton
                icon={ArrowDown}
                label="Abajo"
                onClick={() => handleManual("backward")}
              />
              <ControlButton
                icon={ArrowRight}
                label="Der."
                onClick={() => handleManual("right")}
              />
            </div>
            <p className="mt-2 text-xs text-neutral-400 text-center">
              También puedes usar W/A/S/D o flechas del teclado.
            </p>
          </CardContent>
        </Card>

        {/* IA & Telemetría (Experimental) */}
        <Card className="bg-neutral-800 border-neutral-700">
          <CardHeader>
            <h3 className="text-lg font-medium">
              IA & Telemetría <span className="text-xs opacity-60">(experimental)</span>
            </h3>
          </CardHeader>
          <CardContent>
            <p className="text-sm text-neutral-300 mb-2">
              Aquí podrías integrar un asistente de IA que sugiera rutas óptimas,
              detecte obstáculos en tiempo real o resuma la telemetría.
            </p>
            <Button variant="outline" className="w-full" onClick={() => sendCommand("aiSuggestRoute")}>
              Pedir ruta inteligente
            </Button>
          </CardContent>
        </Card>

        {/* Panel de Logs */}
        <Card className="bg-neutral-800 border-neutral-700">
          <CardHeader>
            <h3 className="text-lg font-medium">Telemetría / Logs</h3>
          </CardHeader>
          <CardContent className="h-40 overflow-auto text-xs font-mono">
            {logs.map((text, i) => (
              <p key={i}>{text}</p>
            ))}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
