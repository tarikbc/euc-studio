// First import: inside Tauri, forward web view errors to the terminal before anything else runs.
import "./lib/boot.ts";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import "./design/global.css";
import { trackInputModality } from "./lib/modality.ts";

trackInputModality();

const root = document.getElementById("root");
if (!root) throw new Error("Missing #root element");

createRoot(root).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
