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
      jointName.textContent = "Model loaded";
      jointDetail.textContent = "Joint metadata unavailable.";
    });
}

document.getElementById("previous-joint")?.addEventListener("click", () => selectJoint(selectedJoint - 1));
document.getElementById("next-joint")?.addEventListener("click", () => selectJoint(selectedJoint + 1));
