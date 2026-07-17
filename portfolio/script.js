const video = document.querySelector("video");
const viewButtons = [...document.querySelectorAll(".view-button")];
const viewPanels = [...document.querySelectorAll("[data-view-panel]")];
const modelViewer = document.getElementById("g1-model");
const mobileDemoOnly = window.matchMedia("(max-width: 600px)");

function showView(requestedView) {
  const view = mobileDemoOnly.matches ? "demo" : requestedView;
  viewButtons.forEach((button) => button.classList.toggle("is-active", button.dataset.view === view));
  viewPanels.forEach((panel) => {
    const active = panel.dataset.viewPanel === view;
    panel.hidden = !active;
  });
  if (view === "model" && modelViewer && !modelViewer.hasAttribute("src")) {
    modelViewer.setAttribute("src", modelViewer.dataset.src);
  }
  if (view === "model") video?.pause();
}

viewButtons.forEach((button) => button.addEventListener("click", () => showView(button.dataset.view)));
showView(new URLSearchParams(window.location.search).get("view") || "demo");
mobileDemoOnly.addEventListener("change", () => showView("demo"));

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

const themeToggle = document.querySelector(".theme-toggle");
const architectureDiagram = document.getElementById("architecture-diagram");
const themeColors = {
  light: "#f4f5f2",
  dark: "#171918"
};
const diagramPalettes = {
  light: {
    bg: "#f4f5f2", panel: "#ffffff", panel2: "#e9ece9", media: "#eef0ed",
    text: "#1b1d1c", muted: "#5d6461", quiet: "#858c88", line: "#d5d8d4",
    accent: "#496b78", accentSoft: "#e2e9eb"
  },
  dark: {
    bg: "#171918", panel: "#202321", panel2: "#222725", media: "#1b1f1d",
    text: "#f0f1ed", muted: "#b3b9b5", quiet: "#8d9590", line: "#3a3e3b",
    accent: "#8fb6c3", accentSoft: "#1d2a2e"
  }
};

function applyDiagramPalette(theme) {
  const svg = architectureDiagram?.contentDocument?.documentElement;
  const colors = diagramPalettes[theme];
  if (!svg || !colors) return;
  Object.entries(colors).forEach(([key, value]) => {
    const property = key === "panel2" ? "panel-2" : key === "accentSoft" ? "accent-soft" : key;
    svg.style.setProperty(`--diagram-${property}`, value);
  });
}

function setTheme(theme, persist = false) {
  if (!themeColors[theme]) return;
  document.documentElement.dataset.theme = theme;
  document.documentElement.style.colorScheme = theme;
  document.querySelector('meta[name="theme-color"]')?.setAttribute("content", themeColors[theme]);
  const nextTheme = theme === "dark" ? "light" : "dark";
  const label = `Switch to ${nextTheme} mode`;
  themeToggle?.setAttribute("aria-label", label);
  themeToggle?.setAttribute("title", label);
  if (persist) localStorage.setItem("theme", theme);
  applyDiagramPalette(theme);
}

themeToggle?.addEventListener("click", () => {
  setTheme(document.documentElement.dataset.theme === "dark" ? "light" : "dark", true);
});
architectureDiagram?.addEventListener("load", () => applyDiagramPalette(document.documentElement.dataset.theme));
setTheme(document.documentElement.dataset.theme || "light");
