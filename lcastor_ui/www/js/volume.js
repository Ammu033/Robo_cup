pcntChange = new ROSLIB.Topic({
	ros : ros,
	name : '/volume/percentChange',
	messageType : 'std_msgs/Int8'
});

function vol_up() {
	var Int8 = new ROSLIB.Message({
		data : 10
	});
	pcntChange.publish(Int8);
	console.log("Volume up");
}

function vol_down() {
	var Int8 = new ROSLIB.Message({
		data : -10
	});
	pcntChange.publish(Int8);
	console.log("Volume down");
}

console.log("Volume control loaded!");
