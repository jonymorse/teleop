const video = document.querySelector("video");
const viewButtons = [...document.querySelectorAll(".view-button")];
const viewPanels = [...document.querySelectorAll("[data-view-panel]")];

function showView(view) {
  viewButtons.forEach((button) => button.classList.toggle("is-active", button.dataset.view === view));
  viewPanels.forEach((panel) => {
    const active = panel.dataset.viewPanel === view;
    panel.hidden = !active;
  });
  if (view === "model") video?.pause();
}

viewButtons.forEach((button) => button.addEventListener("click", () => showView(button.dataset.view)));
showView(new URLSearchParams(window.location.search).get("view") || "demo");

const modelViewer = document.getElementById("g1-model");
const jointName = document.getElementById("joint-name");
const jointDetail = document.getElementById("joint-detail");
const jointIndex = document.getElementById("joint-index");
let joints = [];
let selectedJoint = -1;

const readableJointName = (name) => name.replace(/_joint$/, "").replaceAll("_", " ");

function selectJoint(index) {
  if (!joints.length) return;
  selectedJoint = (index + joints.length) % joints.length;
  const joint = joints[selectedJoint];
  document.querySelectorAll(".joint-hotspot").forEach((hotspot, hotspotIndex) => {
    hotspot.classList.toggle("is-selected", hotspotIndex === selectedJoint);
  });
  jointName.textContent = readableJointName(joint.name);
  jointDetail.textContent = `Axis ${joint.axis.join(", ")} · ${joint.lower.toFixed(2)} to ${joint.upper.toFixed(2)} rad`;
  jointIndex.textContent = `${joint.index} / ${joints.length}`;
}

if (modelViewer) {
  fetch("assets/g1-29dof-joints.json?v=20260714-1")
    .then((response) => response.json())
    .then((data) => {
      joints = data;
      joints.forEach((joint, index) => {
        const hotspot = document.createElement("button");
        hotspot.type = "button";
        hotspot.className = "joint-hotspot";
        hotspot.slot = `hotspot-${joint.index}`;
        hotspot.dataset.position = `${joint.position[0]}m ${joint.position[1]}m ${joint.position[2]}m`;
        hotspot.dataset.normal = "0m 1m 0m";
        hotspot.setAttribute("aria-label", readableJointName(joint.name));
        hotspot.addEventListener("click", () => selectJoint(index));
        modelViewer.append(hotspot);
      });
    })
    .catch(() => {
      jointName.textContent = "Model loaded";
      jointDetail.textContent = "Joint metadata unavailable.";
    });
}

document.getElementById("previous-joint")?.addEventListener("click", () => selectJoint(selectedJoint - 1));
document.getElementById("next-joint")?.addEventListener("click", () => selectJoint(selectedJoint + 1));

const paletteButtons = [...document.querySelectorAll("[data-palette]")];
const paletteControl = document.querySelector(".palette-control");
const architectureDiagram = document.getElementById("architecture-diagram");
const paletteThemeColors = {
  paper: "#e1d8c8",
  graphite: "#090908",
  slate: "#080b0e",
  olive: "#0b0d08"
};
const diagramPalettes = {
  paper: {
    bg: "#e1d8c8", panel: "#eee7dc", panel2: "#e8decd", media: "#d1c8b7",
    text: "#1f1d19", muted: "#5e584f", quiet: "#8b8173", line: "#b8ad9d",
    accent: "#9a5538", accentSoft: "#ead8ca"
  },
  graphite: {
    bg: "#090908", panel: "#111110", panel2: "#161514", media: "#0f0f0d",
    text: "#f3f0e8", muted: "#aca79d", quiet: "#706d66", line: "#3b3934",
    accent: "#d96a3a", accentSoft: "#1c100c"
  },
  slate: {
    bg: "#080b0e", panel: "#10151a", panel2: "#151b21", media: "#0d1217",
    text: "#edf3f5", muted: "#a5b0b4", quiet: "#697378", line: "#34404a",
    accent: "#83a9be", accentSoft: "#111d24"
  },
  olive: {
    bg: "#0b0d08", panel: "#11140d", panel2: "#171a12", media: "#0f130b",
    text: "#eff0e5", muted: "#a8aa98", quiet: "#6c6f5f", line: "#3a3f31",
    accent: "#a2a866", accentSoft: "#191b0e"
  }
};

function applyDiagramPalette(palette) {
  const svg = architectureDiagram?.contentDocument?.documentElement;
  const colors = diagramPalettes[palette];
  if (!svg || !colors) return;
  Object.entries(colors).forEach(([key, value]) => {
    const property = key === "panel2" ? "panel-2" : key === "accentSoft" ? "accent-soft" : key;
    svg.style.setProperty(`--diagram-${property}`, value);
  });
}

function closePaletteMenu() {
  paletteControl?.removeAttribute("open");
}

function setPalette(palette) {
  if (!paletteThemeColors[palette]) return;
  document.body.dataset.palette = palette;
  document.querySelector('meta[name="theme-color"]')?.setAttribute("content", paletteThemeColors[palette]);
  paletteButtons.forEach((button) => {
    const active = button.dataset.palette === palette;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  applyDiagramPalette(palette);
}

paletteButtons.forEach((button) => button.addEventListener("click", () => {
  setPalette(button.dataset.palette);
  closePaletteMenu();
}));
architectureDiagram?.addEventListener("load", () => applyDiagramPalette(document.body.dataset.palette));
document.addEventListener("click", (event) => {
  if (!paletteControl?.contains(event.target)) closePaletteMenu();
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") closePaletteMenu();
});
const requestedPalette = new URLSearchParams(window.location.search).get("palette");
setPalette(paletteThemeColors[requestedPalette] ? requestedPalette : "paper");
