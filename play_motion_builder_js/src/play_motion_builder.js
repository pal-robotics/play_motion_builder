const EditActions = Object.freeze({
	LIST: 0,
	APPEND: 1,
	EDIT: 2,
	COPY_AS_NEXT: 3,
	COPY_AS_LAST: 4,
	REMOVE: 5,
	EDIT_TIME: 6
});
const RunModes = Object.freeze({
	RUN_MOTION: 0,
	GO_TO_STEP: 1
});

/**
 * The class Motionbuilder contains the necessry methods to interface with the play_motion_builder node.
 * 
 * @fires joints-changed Emitted when the list of joints & groups is changed
 * @fires motion-changed Emitted when a change in the motion has been detected
 * @fires motion-ran Emitted when a motion finishes its execution
 * @fires stored-motion Emitted when the YAML text is generated, it sends the generated text as a parameter.
 * 
 * @param {string} host The host at which the rosbridge is listening
 * @param {string} port The port at which the rosbridge is listening
 */
class MotionBuilder {
	constructor(host, port) {
		this.ros = new ROSLIB.Ros({
			url: 'ws://' + host + ':' + port
		});
		// Actions
		this.builder_client = new ROSLIB.ActionClient({
			ros: this.ros,
			actionName: 'play_motion_builder_msgs/BuildMotionAction',
			serverName: '/play_motion_builder_node/build'
		});
		this.build_goal = null;

		this.run_motion_client = new ROSLIB.ActionClient({
			ros: this.ros,
			actionName: 'play_motion_builder_msgs/RunMotionAction',
			serverName: '/play_motion_builder_node/run'
		});

		// Services
		this.list_joints_client = new ROSLIB.Service({
			ros: this.ros,
			serviceType: 'play_motion_builder_msgs/ListJointGroups',
			name: '/play_motion_builder_node/list_joint_groups'
		});
		this.edit_motion_client = new ROSLIB.Service({
			ros: this.ros,
			serviceType: 'play_motion_builder_msgs/EditMotion',
			name: '/play_motion_builder_node/edit_motion'
		});
		this.change_joints_client = new ROSLIB.Service({
			ros: this.ros,
			serviceType: 'play_motion_builder_msgs/ChangeJoints',
			name: '/play_motion_builder_node/change_joints'
		});
		this.store_client = new ROSLIB.Service({
			ros: this.ros,
			serviceType: 'play_motion_builder_msgs/StoreMotion',
			name: "/play_motion_builder_node/store_motion"
		});

		// Params
		this.motion_param = new ROSLIB.Param({
			ros: this.ros,
			name: '/play_motion/motions'
		});

		// Internal variables
		this.motion = {
			motion: {},
			meta: {},
			ros_name: ''
		};
		this.groups = [];
		this.extra_joints = [];
		this.run_goal = null;
	}

	/**
	 * List existing play_motion motions
	 * @param {function} cb - A function that will receive a list of motions as a parameter
	 */
	listAvailableMotions(cb) {
		let motion_list = [];
		this.motion_param.get((motions) => {
			for (const motion in motions) {
				motion_list.push({
					key: motion,
					label: motions[motion].hasOwnProperty("meta") ? motions[motion].meta.name : motion
				});
			}
			cb(motion_list);
		});
	}

	/** 
	 * Start the cretion of a motion.
	 * @fires joints-changed
	 * @fires motion-changed
	 * @param {string} motion - Optional name of the motion to be loaded. If undefined, a new motion is created
	 */
	start(motion) {
		let msg = {
			motion: ''
		};

		if (motion !== undefined) {
			msg.motion = motion;
			this.motion.ros_name = motion;

			// TODO load meta info
		}

		// Call action to start building a motion
		this.build_goal = new ROSLIB.Goal({
			actionClient: this.builder_client,
			goalMessage: msg
		});
		window.setTimeout(() => {
			// Retrieve joints and group
			this.listJointGroups();

			// List the configuration of the current motion
			this.listMotion();
		}, 500); // Wait for the goal to be processed
		this.build_goal.send();
		this.build_goal.on('result', () => {
			this.reset();
		});
	}
	/**
	 * Empty the information hold on the motion variable
	 */
	reset() {
		this.motion = {
			motion: {},
			meta: {},
			ros_name: ''
		};
	}

	/**
	 * List the set of keyframes of a motion. 
	 * @fires motion-changed
	 */
	listMotion() {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.LIST
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Load the information on the groups and extra_joints available. 
	 * @fires joints-changed
	 */
	listJointGroups() {
		this.list_joints_client.callService(new ROSLIB.ServiceRequest({}), (response) => {
			this.groups = response.groups;
			this.extra_joints = response.additional_joints;
			this.emit('joints-changed');
		});
	}

	/**
	 * Add the current position of the robot at the end as a new keyframe. 
	 * @fires motion-changed
	 */
	appendKeyframe() {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.APPEND
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Change the positions of the specified keyframe to the current position of the robot. 
	 * @fires motion-changed
	 * param {number} keyframe_id - Index of the keyframe to be changed
	 */
	editKeyframe(keyframe_id) {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.EDIT,
			step_id: keyframe_id
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Copy the specified keyframe ad the next keyframe in the motion. 
	 * @fires motion-changed
	 * param {number} keyframe_id - Index of the keyframe to be copied
	 */
	copyAsNextKeyframe(keyframe_id) {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.COPY_AS_NEXT,
			step_id: keyframe_id
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Copy the specified keyframe ad the last keyframe in the motion. 
	 * @fires motion-changed
	 * param {number} keyframe_id - Index of the keyframe to be copied
	 */
	copyAsLastKeyframe(keyframe_id) {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.COPY_AS_LAST,
			step_id: keyframe_id
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Remove the specified keyframe. 
	 * @fires motion-changed
	 * param {number} keyframe_id - Index of the keyframe to be removed
	 */
	removeKeyframe(keyframe_id) {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.REMOVE,
			step_id: keyframe_id
		}), (response) => {
			if (response.ok) {
				this.motion.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Change the time taken by the specified keyframe. 
	 * @fires motion-changed
	 * param {number} keyframe_id - Index of the keyframe whose time will be changed
	 * param {number} time_from_last - Time from last keyframe in seconds
	 */
	editKeyframeTime(keyframe_id, time_from_last) {
		this.edit_motion_client.callService(new ROSLIB.ServiceRequest({
			action: EditActions.EDIT_TIME,
			step_id: keyframe_id,
			time: time_from_last
		}), (response) => {
			if (response.ok) {
				this.motion = response.motion;
				this.emit('motion-changed');
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Change the used group for this motion. 
	 * @fires motion-changed
	 * param {string} new_group - Group to be used
	 */
	changeGroup(new_group) {
		this.change_joints_client.callService(new ROSLIB.ServiceRequest({
			group: new_group
		}), (response) => {
			if (response.ok) {
				this.listMotion();
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Add an extra joint to be controlled in the motion. 
	 * @fires motion-changed
	 * param {string} joint - Joint to be added
	 */
	addExtraJoint(joint) {
		this.change_joints_client.callService(new ROSLIB.ServiceRequest({
			joints_to_add: [joint]
		}), (response) => {
			if (response.ok) {
				this.listMotion();
			} else {
				console.log(response.message);
			}
		});
	}
	/**
	 * Remove an extra joint to be controlled in the motion. 
	 * @fires motion-changed
	 * param {string} joint - Joint to be removed
	 */
	removeExtraJoint(joint) {
		this.change_joints_client.callService(new ROSLIB.ServiceRequest({
			joints_to_remove: [joint]
		}), (response) => {
			if (response.ok) {
				this.listMotion();
			} else {
				console.log(response.message);
			}
		});
	}

	/**
	 * Run motion at the specified downshift (time between frames is multiplied by this value). 
	 * @fires motion-ran
	 * param {number} downshift - Downshift coefficient
	 */
	runMotion(downshift) {
		this.run_goal = new ROSLIB.Goal({
			actionClient: this.run_motion_client,
			goalMessage: {
				run_mode: RunModes.RUN_MOTION,
				downshift: downshift
			}
		});
		this.run_goal.send();
		this.run_goal.on('result', () => {
			this.emit('motion-ran');
			this.run_goal = null;
		});
	}
	/**
	 * Stop the motion being executed currently. 
	 */
	stopMotion() {
		if (this.run_goal !== null)
			this.run_goal.cancel();
	}
	/**
	 * Move robot to the position specified. 
	 * @fires motion-ran
	 * param {number} keyframed_id - Index of the keyframe position to put the robot in
	 */
	runToStep(keyframe_id) {
		this.run_goal = new ROSLIB.Goal({
			actionClient: this.run_motion_client,
			goalMessage: {
				run_mode: RunModes.GO_TO_STEP,
				step_id: keyframe_id
			}
		});
		this.run_goal.send();
		this.run_goal.on('result', () => {
			this.emit('motion-ran');
			this.run_goal = null;
		});
	}

	/**
	 * Generate motion file. 
	 * @fires stored-motion
	 * param {string} ros_name- Ros Param name for the motion
	 * param {string} name - Readable name for the motion
	 * param {string} usage - Usage of the motion
	 * param {string} description - description of the motion
	 */
	generateFile(ros_name, name, usage, description) {
		this.store_client.callService(new ROSLIB.ServiceRequest({
			file_path: '',
			ros_name: ros_name,
			meta: {
				name: name,
				usage: usage,
				description: description
			}
		}), (response) => {
			if (response.ok) {
				this.emit('stored-motion', response.message);
			} else {
				console.log(response.message);
			}
		});
	}
}
MotionBuilder.prototype.__proto__ = EventEmitter2.prototype;