let motion_builder = new MotionBuilder(window.location.hostname, 9090);

motion_builder.on('joints-changed', () => {
	// Joints list has changed, reload list of groups and extra_joints
	createGroupsList(motion_builder.groups);
	createExtraList(motion_builder.extra_joints);
});
motion_builder.on('motion-changed', () => {
	// Motion has changed, reload table and check proper group and joints
	createTableHead(motion_builder.motion.motion.joints);
	checkUsedGroup(motion_builder.motion.motion.used_group);
	checkExtraJoint(motion_builder.motion.motion.joints, motion_builder.extra_joints);
	cleanTable();
	for (let i = 0; i < motion_builder.motion.motion.keyframes.length; ++i)
		addKeyframeToTable(motion_builder.motion.motion.keyframes[i], i);
});
motion_builder.on('motion-ran', () => {
	// When a motion has finished
	playBtnStyle();
	enableBtns();
});
motion_builder.on('stored-motion', (motion) => {
	let link = document.createElement('a');
	link.download = 'motion.yaml';
	let blob = new Blob([motion], {
		type: 'text/plain'
	});
	link.href = window.URL.createObjectURL(blob);
	link.click();
});

/*************************************************
 **** Functions called from emited events ********
 *************************************************/

function createTableHead(joints) {
	let table_head = document.getElementById("table-head");
	let joint_cell = "";
	//clear
	table_head.innerHTML = "";
	table_head.insertAdjacentHTML("afterbegin", `<th></th>
								<th class="time-tooltip" id="time-cell">TIME
									<!-- Create icon -->
									<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" class="create-icon">
										<path d="M0 0h24v24H0z" fill="none"/>
										<path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z"/>
									</svg>
									<span class="tooltiptext">Edit time below</span>
								</th>`);
	//update
	for (let i = 0; i < joints.length; i++) {
		joint_cell += `<th>${joints[i]}</th>`;
	}
	document.getElementById("time-cell").insertAdjacentHTML("afterend", joint_cell);
}
function cleanTable() {
	document.getElementById("table-body").innerHTML = "";
}
function addKeyframeToTable(keyframe, index) {
	let table_body = document.getElementById("table-body");
	let new_row = table_body.insertRow(-1);
	//add control_joint_icon
	let control_joint_icon = new_row.insertCell(0);
	control_joint_icon.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" class="control-keyframe-icon" onclick="setupContextMenu(event, ${index})">
										<path d="M0 0h24v24H0z" fill="none"/>
										<path d="M13 7h-2v4H7v2h4v4h2v-4h4v-2h-4V7zm-1-5C6.49 2 2 6.49 2 12s4.49 10 10 10 10-4.49 10-10S17.51 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8z"/>
									</svg>`
	//add time_input
	let time_input = new_row.insertCell(1);
	time_input.innerHTML = `<input type="number" name="time" class="time-input" min="0" step="0.5" 
								   value="${keyframe.time_from_last}" onchange="changeTimeInput(${index}, value)">`
	//add keyframe pose
	for (let i = 0; i < keyframe.pose.length; i++) {
		let keyframe_pose = new_row.insertCell(i + 2);;
		keyframe_pose.innerHTML = `${keyframe.pose[i].toFixed(4)}`; 
	}
}
function createGroupsList(groups) {
	let groups_list = document.getElementById("groups-list");
	//clear 
	groups_list.innerHTML = "";
	//update
	let list_item = "";
	for (let i = 0; i < groups.length; i++) {
		list_item += `<div class="list-item">
							<input type="radio" id="${groups[i]}" name="groups" value="${groups[i]}" onclick="changeUsedGroup('${groups[i]}')">
							<label for="${groups[i]}">${groups[i]}</label>
					  </div>`;
	}
	groups_list.insertAdjacentHTML("afterbegin", list_item);
}
function createExtraList(extra_joints) {
	let extra_list = document.getElementById("extra-list");
	//clear 
	extra_list.innerHTML = "";
	//update
	let list_item = "";
	for (let i = 0; i < extra_joints.length; i++) {
		list_item += `<div class="list-item">
						<input type="checkbox" id="${extra_joints[i]}" name="${extra_joints[i]}" value="${extra_joints[i]}" onclick="switchExtraJoint('${extra_joints[i]}')">
						<label for="${extra_joints[i]}">${extra_joints[i]}</label>
					</div>`;
	}
	extra_list.insertAdjacentHTML("afterbegin", list_item);
}
function checkUsedGroup(group) {
	document.getElementById(group).checked = true;
}
function checkExtraJoint(joints, extra_joints) {
	extra_joints.forEach((item) => {
		if (joints.includes(item))
			document.getElementById(item).checked = true;
		else
			document.getElementById(item).checked = false;
	})
}

/*************************************************
 *********** Functions called from HTML ***********
 *************************************************/

function newMotion() {
	if (!document.getElementById("new-btn").classList.contains('disabled-border')) {
		motion_builder.start();
		enableBtns();
	}
}
function loadMotion(motion){
	motion_builder.start(motion);
	enableBtns();
	closePopup('load-popup', 'btn');
}
function loadMetaDates(meta, ros_name) {
	//Put meta dates of motion to #save-popup
	document.getElementById("ros_name").value = ros_name;
	if (meta.hasOwnProperty("name"))
		document.getElementById("meta_name").value = meta.name;
	if (meta.hasOwnProperty("usage"))
		document.getElementById("meta_usage").value = meta.usage;
	if (meta.hasOwnProperty("description"))
		document.getElementById("meta_description").value = meta.description;
	//Open popup
	openPopup('save-popup');
}
function loadMotionsList(available_motions){
	//Load list
	let motions_list = document.getElementById("motions-list");
	//clear 
	motions_list.innerHTML = "";
	//update
	for (let motion of available_motions) {
		motion = `<li id="${motion.key}" onclick="loadMotion('${motion.key}')">${motion.label}</li>`;
		motions_list.insertAdjacentHTML("afterbegin", motion);
	}
	// Open popup
	openPopup('load-popup');
}
function changeUsedGroup(used_group) {
	motion_builder.changeGroup(used_group);
}
function switchExtraJoint(extra_joint) {
	let checkbox = document.getElementById(extra_joint);
	if (checkbox.checked) {
		motion_builder.addExtraJoint(extra_joint);
	} else {
		motion_builder.removeExtraJoint(extra_joint);
	}
}
function captureKeyframe() {
	if (!document.getElementById("capture-btn").classList.contains('disabled-btn')) {
		motion_builder.appendKeyframe();
	}
}
function setupContextMenu(event, index) {
	if (!document.getElementsByClassName("control-keyframe-icon")[index].classList.contains('disabled-icon')) {
		//Show context menu
		let context_menu = document.getElementById("context");
		let menu_items = `<ul>
							<li id="go-to-pose">Go to position</li>
							<li id="recapture">Recapture Keyframe</li>
							<li class="menu-separator"></li>
							<li id="copy-below">Copy below</li>
							<li id="copy-as-last">Copy as last</li>
							<li id="delete">Delete</li>
						  </ul>`;
		context_menu.innerHTML = menu_items;
		context_menu.style.left = event.clientX + "px";
		context_menu.style.top = event.clientY + "px";
		context_menu.classList.add('slow-show');
		
		/********************* Add functions to list items: *********************/
		//- go to position
		document.getElementById("go-to-pose").addEventListener('click', () => {
			motion_builder.runToStep(index);
			hideContextMenu();
		});
		//- recapture keyframe
		document.getElementById("recapture").addEventListener('click', () => {
			motion_builder.editKeyframe(index);
			hideContextMenu();
		});
		//- copy keyframe below
		document.getElementById("copy-below").addEventListener('click', () => {
			motion_builder.copyAsNextKeyframe(index);
			hideContextMenu();
		});
		//- copy keyframe as last
		document.getElementById("copy-as-last").addEventListener('click', () => {
			motion_builder.copyAsLastKeyframe(index);
			hideContextMenu();
		});
		//- delete keyframe
		document.getElementById("delete").addEventListener('click', () => {
			motion_builder.removeKeyframe(index);
			hideContextMenu();
		});

		//Hide context menu 
		document.addEventListener('click', hideContextMenu);
		event.stopPropagation();
	}
}
function hideContextMenu() {
	let context_menu = document.getElementById("context");
	context_menu.classList.remove('slow-show');
	//clean
	context_menu.innerHTML = "";
}
function changeTimeInput(index, time) {
	motion_builder.editKeyframeTime(index, parseFloat(time));
}
function playMotion() {
	// RUN motion: if play-btn is enabled and if it is not executing right now
	if (!document.getElementById("play-btn").classList.contains('disabled-border') &&
		!document.getElementById("play-btn").classList.contains('stop-btn')) {
		let downshift = parseFloat(document.getElementById('downshift').value);
		motion_builder.runMotion(downshift);
		disableBtns();
		stopBtnStyle();
	// STOP motion: if play-btn is executing right now
	} else if (document.getElementById("play-btn").classList.contains('stop-btn')) {
		motion_builder.stopMotion();
		playBtnStyle();
		enableBtns();
	}
}
function openPopup(id) {
	let popup = document.getElementById(id);
	popup.classList.remove("d-none");
	popup.classList.remove("fadeOut");
	popup.classList.add("fadeIn");
}
function closePopup(id, click) {
	let popup = document.getElementById(id);
	if (click == "btn" || click == id) {
		popup.classList.remove("fadeIn");
		popup.classList.add("fadeOut");
		setTimeout(function() {
			popup.classList.add("d-none");
		}, 300);
	} 
}
function checkRosName(value) {
	//Check if ros_name is valid
	let rules = /^[a-z|A-Z][a-z|A-Z|0-9|_]*$/g;
	let result = rules.test(value);

	//Notify and disable btn #save-btn-popup if ros_name is not valid
	let notification = document.getElementById("notification");
	if (value !== "" && !result) {
		notification.classList.add("notification");
		notification.innerHTML = "Parameters must begin with a letter<br>and contain only letters, numbers or _ .";
		popupSaveBtn(false);
	} else if (value == "" || result) {
		notification.classList.remove("notification");
		notification.innerHTML = "";
		popupSaveBtn(value != ""); // Can't save with empty name
	}
}
function saveMotion() {
	if (!document.getElementById("save-btn-popup").classList.contains('disabled-text')) {
		//Store meta dates
		motion_builder.motion.ros_name = document.getElementById("ros_name").value;
		motion_builder.motion.meta["name"] = document.getElementById("meta_name").value;
		motion_builder.motion.meta["usage"] = document.getElementById("meta_usage").value;
		motion_builder.motion.meta["description"] = document.getElementById("meta_description").value;
		
		//Generate file
		motion_builder.generateFile(motion_builder.motion.ros_name, 
									motion_builder.motion.meta.name, 
									motion_builder.motion.meta.usage, 
									motion_builder.motion.meta.description);
		//Close popup
		closePopup('save-popup', 'btn');
	}
}

/*************************************************
 ****************** Btns Styles ******************
 *************************************************/

function disableBtns() {
	//'new' btn
	document.getElementById("new-btn").classList.add("disabled-border");
	document.querySelector(".white-btn-text").classList.add("disabled-text");
	//'load' btn
	document.getElementById("load-btn").classList.add("disabled-btn");
	//'capture keyframe' btn 
	document.getElementById("capture-btn").classList.add("disabled-btn");
	//'save as' btn
	document.getElementById("save-btn").classList.add("disabled-btn");
	//'control keyframe' icons
	let control_icons = document.getElementsByClassName("control-keyframe-icon");
	for (let i = 0; i < control_icons.length; i++) {
		control_icons[i].classList.add("disabled-icon");
	}
}

function enableBtns() {
	//'new' btn
	document.getElementById("new-btn").classList.remove("disabled-border");
	document.querySelector(".white-btn-text").classList.remove("disabled-text");
	//'load' btn
	document.getElementById("load-btn").classList.remove("disabled-btn");
	//'play' btn
	document.getElementById("play-btn").classList.remove("disabled-border");
	document.querySelector(".play-btn-text").classList.remove("disabled-text");
	document.querySelector(".play-icon").classList.remove("disabled-icon");
	//'capture keyframe' btn 
	document.getElementById("capture-btn").classList.remove("disabled-btn");
	//'save as' btn
	document.getElementById("save-btn").classList.remove("disabled-btn");
	//'control keyframe' icons
	let control_icons = document.getElementsByClassName("control-keyframe-icon");
	for (let i = 0; i < control_icons.length; i++) {
		control_icons[i].classList.remove("disabled-icon");
	}
}
function stopBtnStyle() {
	document.getElementById("play-text").innerHTML = "Stop";
	document.getElementById("play-icon").innerHTML = `<path d="M0 0h24v24H0z" fill="none"/><path d="M6 6h12v12H6z" fill="#b52015"/>`;
	document.getElementById("play-btn").classList.add("stop-btn");
}
function playBtnStyle() {
	document.getElementById("play-text").innerHTML = "Play";
	document.getElementById("play-icon").innerHTML = `<path d="M0 0h24v24H0z" fill="none"/>
													<path d="M10 16.5l6-4.5-6-4.5v9zM12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8z"/>`;
	document.getElementById("play-btn").classList.remove("stop-btn");
}
function popupSaveBtn(valid) {
	let btn = document.getElementById("save-btn-popup");
	if (valid) {
		btn.classList.remove("disabled-text");
	} else {
		btn.classList.add("disabled-text");
	}
}