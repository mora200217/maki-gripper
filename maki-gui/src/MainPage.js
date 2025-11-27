import React from "react";
import "./MainPage.css";

export const MainPage = () => {
  return (
    <div className="dash-container">

      {/* TITLE */}
      <header className="dash-header">
        <h1 className="title">Maki Gripper</h1>
        <p className="subtitle">GUI Prototipo</p>
      </header>

      {/* TWO-COLUMN LAYOUT */}
      <div className="dash-columns">

        {/* LEFT COLUMN → 3D MODEL SPACE */}
        <div className="left-panel">
          <div className="model3d">
            <p></p>
          </div>
        </div>

        {/* RIGHT COLUMN → 3 stacked cards */}
        <div className="right-panel">

          <div className="card">
            
            <h3>Apertura de dedos</h3>
            <p className="value">5cm</p>
          </div>

          <div className="card">
            
            <h3>Ángulo de muñeca</h3>
            <p className="value">10°</p>
          </div>

          <div className="card connections">
            <h3>Conexión de Clientes</h3>

            <div className="conn-row">
              <span>Haptico</span>
              <span className="status red">Offline</span>
            </div>

            <div className="conn-row">
              <span>MetaQuest</span>
              <span className="status green">Online</span>
            </div>

            <div className="conn-row">
              <span>ESP32</span>
              <span className="status red">Offline</span>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};
