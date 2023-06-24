var TOPICS = {
    '/user_speech': 'std_msgs/String',
    '/interface/status': 'std_msgs/String',
    '/interface/enable': 'std_msgs/String',
    '/interface/disable': 'std_msgs/String',
    '/interface/showhome': 'std_msgs/String',
    '/interface/shownreal': 'std_msgs/String',
    '/interface/showexhibit': 'std_msgs/String',
    '/interface/shownavigation': 'std_msgs/String',
    '/interface/showmantenance': 'std_msgs/String',
    '/interface/showwaitcharging': 'std_msgs/String',
    '/interface/showwaitrelease': 'std_msgs/String',
    '/interface/showtours': 'std_msgs/String',
    '/interface/showmap': 'std_msgs/String',
    '/interface/showmodal': 'std_msgs/String',
    '/interface/showmodalclose': 'std_msgs/String',
    '/interface/showmodalcontinue': 'std_msgs/String',
    '/interface/showmodalcontinueclose': 'std_msgs/String',
    '/interface/showmodalInput': 'std_msgs/String',
    '/interface/showmodalInputclose': 'std_msgs/String',
    // '/STT/goal': 'lindimp_stt/STTActionGoal',
    // '/STT/cancel': 'actionlib_msgs/GoalID',
    // '/STT/result': 'lindimp_stt/STTActionResult',
    '/interface/buttonPressed': 'std_msgs/String',
    '/interface/buttonPressedWhileDisabled': 'std_msgs/String',
    '/interface/pageRefreshed': 'std_msgs/String',
    '/planToExec': 'std_msgs/String',
    '/tts/goal': 'pal_interaction_msgs/TtsActionGoal',
    '/tts/result': 'pal_interaction_msgs/TtsActionResult',
    '/tts/cancel': 'actionlib_msgs/GoalID',
    // '/ptu/state': 'sensor_msgs/JointState',
    // '/task_executor/events': 'strands_executive_msgs/TaskEvent',
    '/PNPActionCmd': 'std_msgs/String',
    '/joy_priority': 'std_msgs/Bool',
    '/power/is_emergency': 'std_msgs/Bool',
    '/localization_status': 'lcastor_navigation/LocalizationStatus',
    // '/blacklisted_areas_monitor/event': 'lindimp_navigation/BlacklistedMonitorEvent'
  };

var ACTIONCLIENTS = {
    '/tts': 'pal_interaction_msgs/TtsAction',
  };


var ROS_TOPIC_HANDLES = {};
var ROS_ACTIONCLIENT_HANDLES = {};
var ENABLED = true;

var env = {};
var ros = null;

// read env viariables
$.getJSON("/env.json", function(data){
  env = data
});
// load JS function
loadJS = function(href) {
  var jsLink = $("<script type='text/javascript' src='"+href+"'>");
  $("body").append(jsLink);
};

// load roslib
loadJS("/js/eventemitter2.js");
loadJS("/js/roslib.js");

start_ros();

function start_ros(){
    // Connecting to ROS
    // -----------------
    ros = new ROSLIB.Ros();

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
        //console.log(error);
    console.log('Rosbridge: Error connecting to websocket server, trying to reconnect in 2s...');
    // try to reconnect maybe the server was not ready yet...
    setTimeout(function() {location.reload();}, 2000);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
        //console.log('Connection made!');
    console.log('Rosbridge: Connected to websocket server.');
    });

  ros.on('close', function() {
    console.log('Rosbridge: Connection to websocket server closed.');
  });
    // Create a connection to the rosbridge WebSocket server.
    //ros.connect('ws://10.82.0.83:9090');

  // wait some time to be sure the file env.json is read
  setTimeout(function(){
    ip = env["ROBOT_IP"];
    console.log("connecting to" + 'ws://' + ip + ':9090' )
    ros.connect('ws://' + ip + ':9090');
  }, 1000); // 1 sec

    Subscribe();
}

function Subscribe(){

  // topics
  // console.log("Registering topics..");
  for (const [name, message] of Object.entries(TOPICS)) {
    // console.log(name, message);
    ROS_TOPIC_HANDLES[name] = new ROSLIB.Topic({
        ros: ros,
        name: name,
        messageType: message
      })
  }

  // actionclients
  // console.log("Registering actionclients..");
  for (const [name, action] of Object.entries(ACTIONCLIENTS)) {
    // console.log(name, action);
    ROS_ACTIONCLIENT_HANDLES[name] = new ROSLIB.ActionClient({
        ros: ros,
        serverName: name,
        actionName: action
      })
  }

  ROS_TOPIC_HANDLES['/tts/goal'].subscribe(function(msg) {
      console.log('listener_speech_goal msg.goal='+msg.goal.text);
      if (msg.goal.rawtext.text != "") {
        Show_robot_speech(msg.goal.rawtext.text, msg.goal_id.id, 'nonblock');
      } else{
        Show_robot_speech("TODO check exact format for this case", msg.goal_id.id, 'nonblock');
      }
  });

  ROS_TOPIC_HANDLES['/tts/result'].subscribe(function(msg) {
      console.log('listener_speech_result msg.result='+msg.result);
      Receive_robot_speech_result(msg.result.text, msg.status.goal_id.id,'nonblock');
  });

  ROS_TOPIC_HANDLES['/tts/cancel'].subscribe(function(msg) {
      console.log('listener_speech_result msg.result='+msg);
      Receive_robot_speech_result();
  });

  ROS_TOPIC_HANDLES['/interface/showhome'].subscribe(function(msg) {
    console.log('listener interface show hom msg.data='+msg.data);
    Show_home(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/shownreal'].subscribe(function(msg) {
    console.log('listener interface show nreal msg.data='+msg.data);
    Show_nreal(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showexhibit'].subscribe(function(msg) {
    console.log('listener interface show exhibit msg.data='+msg.data);
    Show_exhibit(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/shownavigation'].subscribe(function(msg) {
    console.log('listener interface show navigation msg.data='+msg.data);
    Show_navigation(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showtours'].subscribe(function(msg) {
    console.log('listener interface show tours msg.data='+msg.data);
    Show_available_tours();
  });

  ROS_TOPIC_HANDLES['/interface/showmap'].subscribe(function(msg) {
    console.log('listener interface show map msg.data='+msg.data);
    // Show_map();
    Show_leaflet_map();
  });

  ROS_TOPIC_HANDLES['/interface/showmantenance'].subscribe(function(msg) {
    console.log('listener interface show mantenance msg.data='+msg.data);
    Show_mant();
  });

  ROS_TOPIC_HANDLES['/interface/showwaitcharging'].subscribe(function(msg) {
    console.log('listener interface show wait charging msg.data='+msg.data);
    Show_waitCharging();
  });

  ROS_TOPIC_HANDLES['/interface/showwaitrelease'].subscribe(function(msg) {
    console.log('listener interface show wait release msg.data='+msg.data);
    Show_waitRelease();
  });

  ROS_TOPIC_HANDLES['/interface/showmodal'].subscribe(function(msg) {
    console.log('listener interface show modal msg.data='+msg.data);
    Show_modal(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showmodalclose'].subscribe(function(msg) {
    console.log('listener interface show modal msg.data='+msg.data);
    Close_modal(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showmodalcontinue'].subscribe(function(msg) {
    console.log('listener interface show modalContinue msg.data='+msg.data);
    Show_modalContinue(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showmodalcontinueclose'].subscribe(function(msg) {
    console.log('listener interface show modalConinue msg.data='+msg.data);
    Close_modalContinue(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showmodalInput'].subscribe(function(msg) {
    console.log('listener interface show modalInput msg.data='+msg.data);
    Show_modalInput(msg.data);
  });

  ROS_TOPIC_HANDLES['/interface/showmodalInputclose'].subscribe(function(msg) {
    console.log('listener interface show modalInput msg.data='+msg.data);
    Close_modalInput(msg.data);
  });


  // ROS_TOPIC_HANDLES['/STT/goal'].subscribe(function(msg) {
  //   console.log('listener /STT/goal msg.goal.msg='+ msg.goal.msg);
  //   Show_listening_alert(msg.goal.msg);
  // });

  // ROS_TOPIC_HANDLES['/STT/result'].subscribe(function(msg) {
  //   console.log('listener /STT/resutl  msg.result.transcription='+ msg.result.transcription);
  //   Close_listening_alert();
  // });

  // ROS_TOPIC_HANDLES['/STT/cancel'].subscribe(function(msg) {
  //   console.log('listener /STT/cancel  msg.result.transcription='+ msg);
  //   Close_listening_alert();
  // });

  // ROS_TOPIC_HANDLES['/task_executor/events'].subscribe(function(msg) {
  //   console.log('listener /task_executor/events msg='+ msg);
  //   Notify_task_event(msg);
  // });

  ROS_TOPIC_HANDLES['/PNPActionCmd'].subscribe(function(msg) {
    console.log('listener /PNPActionCmd msg.data='+ msg.data);
    Notify_PNP_action(msg.data);
  });

  ROS_TOPIC_HANDLES['/joy_priority'].subscribe(function (msg) {
    console.log('listener /joy_priority msg.data=' + msg.data);
    if (msg.data) {
      Notify_bumper_pressed();
    } else {
      Notify_bumper_released();
    }
  });


  ROS_TOPIC_HANDLES['/power/is_emergency'].subscribe(function (msg) {
    console.log('listener /power/is_emergency msg.data=' + msg.data);
    if (msg.data) {
      Notify_EB_pressed();
    } else {
      Notify_EB_released();
    }
  });

  // ROS_TOPIC_HANDLES['/motor_status'].subscribe(function(msg) {
  //     if (msg.emergency_button_pressed) {
  //         //console.log('motor_status listener EB pressed');
  //         Notify_EB_pressed();
  //     } else {
  //         Notify_EB_released();
  //     }

  //     if (msg.bumper_pressed) {
  //         console.log('motor_status listener bumper pressed');
  //         Notify_bumper_pressed();
  //     //} else if (msg.motor_stopped && !msg.emergency_button_pressed) {
  //     //    console.log('motor_status listener bumper pressed');
  //     //    Notify_bumper_pressed();
  //     } else if (!msg.motor_stopped) {
  //        console.log('motor_status listener bumper released')
  //        Notify_bumper_released();
  //     }
  // });
  // ROS_TOPIC_HANDLES['/virtual_bumper_event'].subscribe(function(msg) {
  //     if (msg.freeRunStarted) {
  //        console.log('virtual_bumper_event listener bumper released')
  //        Notify_bumper_released();
  //     }
  // });
  ROS_TOPIC_HANDLES['/localization_status'].subscribe(function(msg) {
      if (msg.status == 0) {
          console.log('localization not accurate notification');
          Notify_localization_bad();
      } else {
          Notify_localization_good();
      }
  });
  // ROS_TOPIC_HANDLES['/blacklisted_areas_monitor/event'].subscribe(function(msg) {
  //   if (msg.status == 1) {
  //       console.log('navigating into blacklisted area!');
  //       Notify_blocked_area();
  //   } else {
  //       Notify_unblocked_area();
  //   }
  // });

  console.log("js subscribed to ros topics")
}

var time_disabled = []
var mouseMovedSinceBtnPressed = true
var timeBtnPressed = Date.now();
var pageChangeDisabled = false;

// Encapsulates the behaviour of a pressed button that will start some physical behaviour (e.g. navigation)
function SafePhysicalButtonEvent(func, id) {
  Check_exhist_param("/navigation_enabled").then((params) => Check_param_value(params).then(value => {
    if (value) {
      SafeButtonEvent(func, id);
    }
  }));
}

// Encapsulates the behaviour of a pressed button by considering the interface_enabled param
// and avoiding too many repetitive clicks
function SafeButtonEvent(func, id) {
    Check_exhist_param("/interface_enabled").then((params) => Check_param_value(params).then(value => {
      if (value && !time_disabled[id] && !pageChangeDisabled) {
          time_disabled[id] = true;
          timeBtnPressed = Date.now();
          mouseMovedSinceBtnPressed = false
          func()
          Signal_buttonPressed(id);
          setTimeout(function(){ time_disabled[id] = false}, 1000);
      }
      if (!value) {
          timeBtnPressed = Date.now();
          Signal_buttonPressed_whileDisabled(id);
      }
    }));
}

// Encapsulates the behaviour of a pressed button avoiding too many repetitive clicks
function SafeButtonEvent_noDisable(func, id) {
    //alert(mouseMovedSinceBtnPressed);
    if (!time_disabled[id] && !pageChangeDisabled) {
        timeBtnPressed = Date.now();
        time_disabled[id] = true
        mouseMovedSinceBtnPressed = false
        func()
        Signal_buttonPressed(id);
        setTimeout(function(){ time_disabled[id] = false}, 2000);
    }
}

// without this if the mouse is left on a button mouseover is continuously triggered when page updates
function Signal_mousemoved() {
    // we don't want to signal also the movements that triggered the btn pressing itself
    if ((Date.now() - timeBtnPressed) > 500)
        mouseMovedSinceBtnPressed = true;
    //alert("moved")
}

function Signal_pageChange() {
    pageChangeDisabled = true;
    console.log("page change disable enabled");
    setTimeout(function(){ pageChangeDisabled = false; console.log("page change disabled");}, 1000);
}

function Command_speech(txt){
    //Show_speech(txt,'nonblock');
    var str_target=txt;

    speak_goal_action = new ROSLIB.Goal({
        actionClient : ROS_ACTIONCLIENT_HANDLES["/speak"],
        goalMessage : {
            text : str_target
        }
    });

    // Then we create the payload to be published. The object we pass in to ros.Message matches the
    console.log('goal_action.send() ' );

    // And finally, publish.
    speak_goal_action.send();
    speak_goal_action.on('result', function(result) {
        console.log('speak_goal_action Final Result: ' ); //result.state sensor_msgs/JointState
    });
}

function Signal_buttonPressed(button) {
  console.log('Signal_buttonPressed' + button);
  msg = new ROSLIB.Message({
    data: button
  });

  ROS_TOPIC_HANDLES['/interface/buttonPressed'].publish(msg);
}

function Signal_buttonPressed_whileDisabled(button) {
  console.log('Signal_buttonPressed_whileDisabled' + button);
  msg = new ROSLIB.Message({
    data: button
  });

  ROS_TOPIC_HANDLES['/interface/buttonPressedWhileDisabled'].publish(msg);
}

function Signal_refreshPage(data) {
  console.log('Signal_refreshpage' + data);
  msg = new ROSLIB.Message({
    data: data
  });

  ROS_TOPIC_HANDLES['/interface/pageRefreshed'].publish(msg);
}

function Interface_status(str){
    var str_goal=""+str+"";
    //log_html('Published message ' + str_goal);
    msg = new ROSLIB.Message({
            data: str_goal
    });
    // Then we create the payload to be published. The object we pass in to ros.Message matches the

    // And finally, publish.
    ROS_TOPIC_HANDLES['/interface/status'].publish(msg);
}

function Alternative_input(sentence) {
  console.log("publishing" + sentence);
  var str_sentence = ""+sentence+""; 
  msg = new ROSLIB.Message({
    data: str_sentence
  });

  // And finally, publish.
  ROS_TOPIC_HANDLES['/user_speech'].publish(msg);
}

function Start_tour_task(tour_key, duration=60*60) {
  console.log("start tour: " + tour_key);

  // Demand_task("guided_tour", ["tours_main", tour_key], "", duration)
  Demand_task("guided_tour", ["learned_tour", tour_key], "", duration)
}

function Start_describe_task(exh_key, duration=60*5) {
  console.log("start describe: " + exh_key);

  Demand_task("describe_exhibit", [exh_key], "", duration)
}

function Start_gotoAndDescribe_task(exh_key, duration=60*30) {
  console.log("start describe: " + exh_key);

  Demand_task("goto_and_describe", [exh_key], "", duration)
}

function Demand_task(action, parameters, waypoint, duration) {
  overlay_off();
  overlay_on();
  console.log("demand task: " + action + " " + parameters + " " + waypoint + " " + duration);
  var service = new ROSLIB.Service({
    ros: ros,
    name: '/task_executor/demand_task',
    serviceType: 'strands_executive_msgs/DemandTask'
  });
  var argument_list = [];
  console.log(parameters);
  if (parameters.length > 0) {
    argument_list[0] = {first: "\"____str____\"", second: parameters[0]};
    if (parameters.length > 1) {
      argument_list[1] = {first: "\"____str____\"", second: parameters.slice(1).join()};
    }
  }
  console.log(argument_list);

  var request = new ROSLIB.ServiceRequest({
    task : {
      action : action,
      start_node_id : waypoint,
      expected_duration : {secs: duration, nsecs: 0},
      max_duration : {secs: duration*2, nsecs:0},
      arguments : argument_list,
      priority : 3
    }
  });


  service.callService(request, function(result) {
    console.log('Task demand result: ');
    console.log(result)

    overlay_off();

    // show the footer that allows to stop the task
    // if (result.success == true) {
    //   Enable_stopTask_btn();
    // }
  }, function(error) {
    console.log('Task demand error: ');
    console.log(error)

    overlay_off();

  });
}

function Cancel_active_task() {
  overlay_off();
  overlay_on();
  console.log("stopping active tasks");
  var service = new ROSLIB.Service({
    ros: ros,
    name: '/task_executor/cancel_active_task',
    serviceType: 'strands_executive_msgs/CancelActiveTask'
  });

  service.callService(new ROSLIB.ServiceRequest({}), function(result) {
    console.log('Cancel active task result: ');
    console.log(result)


    overlay_off();
    // if (result.cancelled == true) {
    //   Disable_stopTask_btn();
    // }
  }, function(error) {  
    console.log('Cancel active task error: ');
    console.log(error)

    overlay_off();

  });
}

const Check_exhist_param = (par_name) => new Promise(resolve => {
    ros.getParams(function(list) {
      var exhists = false;

      for (var i=0; i<list.length; i++) {
        if (list[i] == par_name)
          exhists = true;
      }
      // console.log("exhists: " + exhists)
      resolve([par_name, exhists]);
    });
  });

  // params is a vector which contains the parameter name (string) and if the parameter exhists already (0/1)
const Check_param_value = (params) => new Promise(resolve => {
  par_name = params[0];
  exhists = params[1];
    if (exhists) {
      var enabled = new ROSLIB.Param({
        ros : ros,
        name : par_name
      });
      enabled.get(function(val) {
        // console.log("raw_val " + val)
        resolve(val);
      });
    } else {
      resolve(1);
    }
  });

// Check_exhist_param("/interface_enabled").then((name, exhists) => Check_param_value(name, exhists).then(value => console.log("check final value: " + value)));
