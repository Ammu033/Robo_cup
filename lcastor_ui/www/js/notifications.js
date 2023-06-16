// load JS function
loadJS = function(href) {
  var jsLink = $("<script type='text/javascript' src='"+href+"'>");
  $("body").append(jsLink);
};

// load speech_tools js
loadJS("/js/speechbubble.js");

function Show_navigation_feedback(waypoint) {
  console.log('showing nav feedback')
}

// List of speech bubbles to populate
var $speech_bubbles = {}

// show the robot's speech dialog bubble
function Show_robot_speech(str, id, mode) {
  console.log("removed all the speech bubbles")
  $speech_bubbles[id] = robot_speech_bubble(str);
  $speech_bubbles[id].slideDown();
  $speech_bubbles[id].attr("style", "display: block");
  console.log("open bubble" + id)
}

// close the robot's dialog bubble when the robot finished to speak
function Receive_robot_speech_result(str, id, mode) {
  if(mode=='nonblock' && id in $speech_bubbles) {
    $speech_bubbles[id].slideUp();
    $speech_bubbles[id].remove()
    delete $speech_bubbles[id];
    console.log("removed bubble" + id)
  } else {
    for (var i=0; i<$speech_bubbles.length; i++) {
      $speech_bubbles[i].slideUp();
      $speech_bubbles[i].remove()
      delete $speech_bubbles[i];
    }
    console.log("removed all the speech bubbles")
  }
}

// initialize all the toasts
$('.toast').toast()
$('.toast').toast('show');

var EB_alert_display = true;
function Notify_EB_pressed() {
    if (!EB_alert_display) {
        $("[role=EB-alert]").toast('show');
        EB_alert_display = true;
    }
}
function Notify_EB_released() {
    if (EB_alert_display) {
        $("[role=EB-alert]").toast('hide');
        EB_alert_display = false;
    }
}

var localization_alert_display = true;
function Notify_localization_bad() {
    if (!localization_alert_display) {
        $("[role=localization-alert]").toast("show");
        localization_alert_display = true;
    }
}
function Notify_localization_good() {
    if (localization_alert_display) {
        $("[role=localization-alert]").toast('hide');
        localization_alert_display = false;
    }
}

var blocked_alert_display = true;
function Notify_blocked_area() {
    if (!blocked_alert_display) {
        $("[role=blocked-alert]").toast("show");
        blocked_alert_display = true;
    }
}
function Notify_unblocked_area() {
    if (blocked_alert_display) {
        $("[role=blocked-alert]").toast('hide');
        blocked_alert_display = false;
    }
}

var bumper_alert_display = true;
function Notify_bumper_pressed() {
    if (!bumper_alert_display) {
        $("[role=bumper-alert]").toast("show");
        bumper_alert_display = true;
    }
}
function Notify_bumper_released() {
    if (bumper_alert_display) {
        $("[role=bumper-alert]").toast('hide');
        bumper_alert_display = false;
    }
}

function Show_modal(text) {
  $("[role=modal]").load("modal.html", function() {
    $('[role=dialog]').modal({
      backdrop: 'static',
      keyboard: false,
      focus: true
    });
    overlay_off();
    $('.modal-title').html(text.split("_").join(" "));
    $('[role=dialog]').modal('show');

    $('#no_btn').mousedown(function(){
      Signal_buttonPressed("modalNo");
    });
    $('#yes_btn').mousedown(function(){
      Signal_buttonPressed("modalYes");
    });
    // $("body").prepend(modal_dialog);
  });
  console.log("showing dialog");
}

function Show_modalContinue(text) {
  $("[role=modal]").load("modalContinue.html", function() {
    $('[role=dialog]').modal({
      backdrop: 'static',
      keyboard: false,
      focus: true
    });
    overlay_off();
    $('.modal-title').html(text.split("_").join(" "));
    $('[role=dialog]').modal('show');

    $('#continue_btn').mousedown(function(){
      Signal_buttonPressed("modalContinue");
    });
    // $("body").prepend(modal_dialog);
  });
  console.log("showing dialog continue");
}

function Show_modalInput(text) {
  $("[role=modal]").load("modalInput.html", function() {
    $('[role=dialog]').modal({
      backdrop: 'static',
      keyboard: false,
      focus: true
    });
    overlay_off();
    $('.modal-title').html(text.split("_").join(" "));
    $('[role=dialog]').modal('show');

    $('#submit_btn').click(function(event){
      var sentence = $('#InputText').val();
      console.log("submitted" + sentence);
      Alternative_input(sentence);
      Signal_buttonPressed("modalInput");
      Close_modalInput();
    });
    // $("body").prepend(modal_dialog);
  });
  console.log("showing dialog continue");
}

function Close_modal(text) {
  $('[role=dialog]').modal('hide');
  console.log("hiding dialog");
}

function Close_modalContinue(text) {
  $('[role=dialog]').modal('hide');
  console.log("hiding dialog");
}

function Close_modalInput(text) {
  $('[role=dialog]').modal('hide');
  console.log("hiding dialog");
}


function Show_listening_alert(text) {
  text = "I am listening, speak to me! ";
  $('[role=listening-alert]').load("listening-alert.html", function() {
    $('[role=alert-text]').html(text);
    console.log("opening the speech alert");
  })
}

function Close_listening_alert() {
  $('[role=listening-bs-alert]').alert('close');
  console.log("closing the speech alert");
}

function Notify_task_event(msg) {
  var action = msg.task.action;
  var priority = msg.task.priority;
  var event = Get_event_desc(msg.event);

  $("[role=task-name-text]").text(action);
  $("[role=task-priority-text]").text(priority);
  $("[role=task-event-text]").text(event);

  if (action != "/roam_task_server") {
    // do not allow to stop tasks with priority > 100 from the interface
    if (event == "TASK_STARTED" && priority < 100) {
      Enable_stopTask_btn();
    } else if (event == "TASK_FAILED" || event == "TASK_PREEMPTED" || event == "TASK_SUCCEEDED") {
      Disable_stopTask_btn();
    }
  }

  window.localStorage.setItem('task', action);
  window.localStorage.setItem('priority', priority);
  window.localStorage.setItem('event', event);
}

function Notify_PNP_action(msg) {
  $("[role=action-text]").text(msg);

  window.localStorage.setItem('action', msg);
}

function Get_event_desc(event_number) {
  if (event_number == 1)
    return "ADDED"
  else if (event_number == 2)
    return "DEMANDED"
  else if (event_number == 3)
    return "TASK_STARTED"
  else if (event_number == 4)
    return "NAVIGATION_STARTED"
  else if (event_number == 5)
    return "NAVIGATION_SUCCEEDED"
  else if (event_number == 6)
    return "NAVIGATION_FAILED"
  else if (event_number == 7)
    return "NAVIGATION_PREEMPTED"
  else if (event_number == 8)
    return "EXECUTION_STARTED"
  else if (event_number == 9)
    return "EXECUTION_SUCCEEDED"
  else if (event_number == 10)
    return "EXECUTION_FAILED"
  else if (event_number == 11)
    return "EXECUTION_PREEMPTED"
  else if (event_number == 12)
    return "CANCELLED_MANUALLY"
  else if (event_number == 13)
    return "DROPPED"
  else if (event_number == 14)
    return "TASK_FINISHED"
  else if (event_number == 15)
    return "TASK_FAILED"
  else if (event_number == 16)
    return "TASK_SUCCEEDED"
  else if (event_number == 17)
    return "TASK_PREEMPTED"
  else if (event_number == 30)
    return "ROUTINE_STARTED"
  else if (event_number == 31)
    return "ROUTINE_STOPPED"
}

function Enable_stopTask_btn() {
  console.log("Enabling stop_task_button")
  $("[role=stop-tasks-btn]").removeAttr("disabled")
  window.localStorage.setItem("stop-tasks-btn", "enabled")
}

function Disable_stopTask_btn() {
  console.log("Disabling stop_task_button")
  $("[role=stop-tasks-btn]").attr("disabled", "");
  window.localStorage.setItem("stop-tasks-btn", "disabled")
}

function overlay_on(spinner=true) {

  document.getElementById("overlay").style.display = "block";

  if (spinner) {
      var opts = {
        lines: 13, // The number of lines to draw
        length: 38, // The length of each line
        width: 17, // The line thickness
        radius: 45, // The radius of the inner circle
        scale: 0.3, // Scales overall size of the spinner
        corners: 1, // Corner roundness (0..1)
        color: '#ffffff', // CSS color or array of colors
        fadeColor: 'transparent', // CSS color or array of colors
        speed: 0.8, // Rounds per second
        rotate: 0, // The rotation offset
        animation: 'spinner-line-fade-quick', // The CSS animation name for the lines
        direction: 1, // 1: clockwise, -1: counterclockwise
        zIndex: 2e9, // The z-index (defaults to 2000000000)
        className: 'spinner', // The CSS class to assign to the spinner
        top: '50%', // Top position relative to parent
        left: '50%', // Left position relative to parent
        shadow: '0 0 1px transparent', // Box-shadow for the lines
        position: 'absolute' // Element positioning
      };

      var target = document.getElementById('overlay');
      var spinner = new Spinner(opts).spin(target);
  }
}

function overlay_off() {
  document.getElementById("overlay").style.display = "none";
}
