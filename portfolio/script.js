const video = document.querySelector("video");
if (video) {
  video.addEventListener("play", () => {
    document.querySelector(".demo-panel")?.classList.add("is-playing");
  }, { once: true });
}

const tabs = [...document.querySelectorAll(".tab")];
const panels = [...document.querySelectorAll(".tab-panel")];

function selectTab(tab) {
  tabs.forEach((item) => {
    const active = item === tab;
    item.classList.toggle("is-active", active);
    item.setAttribute("aria-selected", String(active));
  });

  panels.forEach((panel) => {
    const active = panel.id === tab.getAttribute("aria-controls");
    panel.classList.toggle("is-active", active);
    panel.hidden = !active;
  });
}

tabs.forEach((tab, index) => {
  tab.addEventListener("click", () => selectTab(tab));
  tab.addEventListener("keydown", (event) => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
    event.preventDefault();
    const direction = event.key === "ArrowRight" ? 1 : -1;
    const next = tabs[(index + direction + tabs.length) % tabs.length];
    next.focus();
    selectTab(next);
  });
});

const viewButtons = [...document.querySelectorAll(".view-button")];
const viewPanels = [...document.querySelectorAll("[data-view-panel]")];

viewButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const view = button.dataset.view;
    viewButtons.forEach((item) => item.classList.toggle("is-active", item === button));
    viewPanels.forEach((panel) => {
      const active = panel.dataset.viewPanel === view;
      panel.classList.toggle("is-active", active);
      panel.hidden = !active;
    });
    if (view === "model") video?.pause();
  });
});

const requestedView = new URLSearchParams(window.location.search).get("view");
if (requestedView) {
  document.querySelector(`.view-button[data-view="${requestedView}"]`)?.click();
}

const modelViewer = document.getElementById("g1-model");
const jointName = document.getElementById("joint-name");
const jointDetail = document.getElementById("joint-detail");
const jointIndex = document.getElementById("joint-index");
let joints = [];
let selectedJoint = -1;

function readableJointName(name) {
  return name.replace(/_joint$/, "").replaceAll("_", " ");
}

function selectJoint(index) {
  if (!joints.length) return;
  selectedJoint = (index + joints.length) % joints.length;
  const joint = joints[selectedJoint];
  document.querySelectorAll(".joint-hotspot").forEach((hotspot, hotspotIndex) => {
    hotspot.classList.toggle("is-selected", hotspotIndex === selectedJoint);
  });
  jointName.textContent = readableJointName(joint.name);
  jointDetail.textContent = `Axis ${joint.axis.join(", ")} · range ${joint.lower.toFixed(2)} to ${joint.upper.toFixed(2)} rad`;
  jointIndex.textContent = `${joint.index} / ${joints.length}`;
}

if (modelViewer) {
  fetch("assets/g1-29dof-joints.json")
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
      jointName.textContent = "Model available";
      jointDetail.textContent = "Joint metadata could not be loaded.";
    });
}

document.getElementById("previous-joint")?.addEventListener("click", () => selectJoint(selectedJoint - 1));
document.getElementById("next-joint")?.addEventListener("click", () => selectJoint(selectedJoint + 1));
