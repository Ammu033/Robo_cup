listeners["getTaskEvent"] = rwcListenerGetTaskEvent;

var eventDescs = [
    "ADDED",
    "DEMANDED",
    "TASK_STARTED",
    "NAVIGATION_STARTED",
    "NAVIGATION_SUCCEEDED",
    "NAVIGATION_FAILED",
    "NAVIGATION_PREEMPTED",
    "EXECUTION_STARTED",
    "EXECUTION_SUCCEEDED",
    "EXECUTION_FAILED",
    "EXECUTION_PREEMPTED",
    "CANCELLED_MANUALLY",
    "DROPPED",
    "TASK_FINISHED",
    "TASK_FAILED",
    "TASK_SUCCEEDED",
    "TASK_PREEMPTED",
    "ROUTINE_STARTED",
    "ROUTINE_STOPPED"
];

// Listener function 'rwcListenerGetTaskEvent'
async function rwcListenerGetTaskEvent(listenerComponent = null){
    // Topic info loaded from rwc-config JSON file
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : configJSON.listeners["task-event"].topicName,
      messageType : configJSON.listeners["task-event"].topicMessageType
    });

    var rwcTaskEvent = await subTaskEvent(listener, listenerComponent) -1;

    return rwcTaskEvent;
}

// Promise returns value 50ms after subscribing to topic,
// preventing old or undefined values from being returned
function subTaskEvent(listener, listenerComponent = null){
    return new Promise(function(resolve) {
        listener.subscribe(function(message) {
            var rwcTaskEvent = eventDescs[message.event - 1];
            if (listenerComponent === null){
                listener.unsubscribe();
            }
            else if (listenerComponent.dataset.live === "false"){
                listenerComponent.shadowRoot.innerHTML = "<span>" + rwcTaskEvent + "</span>";
                listener.unsubscribe();
            }
            else {
                listenerComponent.shadowRoot.innerHTML = "<span>" + rwcTaskEvent + "</span>";
            }
            setTimeout(function(){
                resolve(rwcTaskEvent);
            }, 50);
        });
    });
}



$(document).ready(function(){
    var motorStatusTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/motor_status",
        messageType: "scitos_msgs/MotorStatus"
    });

    motorStatusTopic.subscribe(function(msg){
        var ePressed = msg.emergency_button_pressed;
        if (ePressed){
            $("#EB-alert").show();
        } else {
            $("#EB-alert").hide();
        }
    });

    stopButton.hidden = true;
});