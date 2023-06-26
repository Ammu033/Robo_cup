$.when(
  $.getStylesheet("/widgets/flipclock/flipclock.css"),
  $.getStylesheet("/css/custom-flipclock.css"),
  $.getScript("/widgets/flipclock/flipclock.min.js"),
  $.Deferred(function( deferred ){
    $( deferred.resolve );
  })
).done(function(){

  // import * as Ladda from 'ladda';

  $("[role=header]").load("/header.html", function(){

    Check_exhist_param("/_next_tour_starting_time").then((params) => Check_param_value(params).then(value => {
      var current_time = Date.now() / 1000 // in seconds
      var diff = value - current_time
      // console.log(current_time)
      // console.log(value)
      // console.log(diff)
      // var date = new Date(value*1000);
      // // Hours part from the timestamp
      // var hours = date.getHours();
      // // Minutes part from the timestamp
      // var minutes = "0" + date.getMinutes();
      // // Seconds part from the timestamp
      // var seconds = "0" + date.getSeconds();
      //
      // // Will display time in 10:30:23 format
      // var formattedTime = hours + ':' + minutes.substr(-2) + ':' + seconds.substr(-2);
      // console.log(formattedTime)
      if (diff > 0) {
        $("[role=fc_text]").html("Next tour in:")
        // add the flipclock
        var clock = new FlipClock($('.tour-countdown'), {
          // ... your options here
        });
        clock.setTime(diff);  //1 hour 3600
        clock.setCountdown(true);
      }
    }));


    // event to home button
    $("#home_page").mouseover(function(){
        SafeButtonEvent(function() {
            console.log("HOME")
            Show_home();
        }, "home")
    });
  });

});
