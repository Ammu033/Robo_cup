$("[role=footer]").load("/footer.html", function(){


    // footer
    $("[role=stop-tasks-btn]").mouseover( function(){
        SafeButtonEvent_noDisable(function(){
            Cancel_active_task();
        }, "cancel");
    })

    if (window.localStorage.getItem("stop-tasks-btn") == "enabled") {
      Enable_stopTask_btn()
    }

    var task = window.localStorage.getItem('task');
    var priority = window.localStorage.getItem('priority');
    var event = window.localStorage.getItem('event');
    var action = window.localStorage.getItem('action');

    $("[role=task-name-text]").text(task);
    $("[role=task-priority-text]").text(priority);
    $("[role=task-event-text]").text(event);
    $("[role=action-text]").text(action);
});
