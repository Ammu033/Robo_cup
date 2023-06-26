/**
 * Speech_bubble - GUI text-to-speech Plugin by robpinillos@gmail.com
 *
 */
(function() {


  // Define constructor
  this.Bubble = function() {

    // Create global element references
    this.closeButton = null;
    this.bubble = null;
    this.overlay = null;

    // Define option defaults
    var defaults = {
			object:'',
			source: 'robot', //['robot', 'user']
			type: 'speaking', // ['speaking','thinking']
			speechRecognition: true,
			origin: "top-right", // ["top-right","top-left","bottom-right","bottom-left"]
      content: "",
      maxWidth: "90%",
      minWidth: "20%",
			minHeight: "5%",
      align: "center",
			overlay: true
    }

    // Create options by extending defaults with the passed in arugments
    if (arguments[0] && typeof arguments[0] === "object") {
      this.options = extendDefaults(defaults, arguments[0]);
    }

		if(this.options.overlay==true) {
			 this.overlay = jQuery('<div id="overlay"> </div>');
		 }

  }

  // Utility method to extend defaults with user options
  function extendDefaults(source, properties) {
    var property;
    for (property in properties) {
      if (properties.hasOwnProperty(property)) {
        source[property] = properties[property];
      }
    }
    return source;
  }

	// Public Methods
  Bubble.prototype.close = function() {

		this.bubble.slideUp();
  }

  Bubble.prototype.open = function(text) {
			console.log(this.bubble.attr('id'));
			this.bubble.empty();
		  this.bubble.text(String(text)).css({
				top: 40 ,
				left: 10
		  });

    this.bubble.slideDown();
    this.bubble.css("display","block");
  }

	// Private Methods

  function buildDiv() {
		// Create modal element
    this.bubble=$(this.object);
		console.log(this.bubble);
    this.bubble.addClass(this.type+"-"+this.origin);
		this.bubble.css("max-width",this.maxWidth);
		this.bubble.css("min-width",this.minWidth);
		this.bubble.css("min-height",this.minHeight);


    // If closeButton option is true, add a close button
    if (this.closeButton === true) {

    }


    	if(this.type === "thinking"){

			}

	}


	buildDiv();

	Bubble.prototype.init_recognition= function() {


  this.overlay.appendTo(document.body);

	var div=$('<div>');
		div.append(
		$('<div>').attr('class','ui-grid-a').append(
			$('<div>').attr('class','ui-block-a').attr('style','width:15%;').append(
				$('<img>').attr('id','start_img').attr('src','./widgets/speech_tools/images/mic-animate.gif')
			),
			$('<div>').attr('class','ui-block-b').attr('style','width:80%;').append(
				$('<div>').attr('id','speech2text').html('Listening ...')
			)
		)

	).enhanceWithin();

    start_stop_Button(event);
	return div;
}

}(jQuery));



var final_transcript = '';
var recognizing = false;
var ignore_onend;
var start_timestamp;
var start_img;


if (!('webkitSpeechRecognition' in window)) {
  upgrade();
} else {
  var recognition = new webkitSpeechRecognition();
  recognition.continuous = true;
  recognition.interimResults = true;

  recognition.lang = "en-GB";

  recognition.onstart = function() {
    recognizing = true;
    start_img.src = './widgets/speech_tools/images/mic-animate.gif';
  };


  recognition.onerror = function(event) {
    if (event.error == 'no-speech') {
      start_img.src = './widgets/speech_tools/images/mic.gif';
      showInfo('info_no_speech');
      ignore_onend = true;
    }
    if (event.error == 'audio-capture') {
      start_img.src = './widgets/speech_tools/images/mic.gif';
      showInfo('info_no_microphone');
      ignore_onend = true;
    }
    if (event.error == 'not-allowed') {
      var event_timeStamp =new Date().getTime();
      if (event_timeStamp - start_timestamp < 100) {
        showInfo('info_blocked');
      } else {
        showInfo('info_denied');
      }
      ignore_onend = true;
    }
  };

  recognition.onend = function() {
    recognizing = false;
    if (ignore_onend) {
      return;
    }
    start_img.src = './widgets/speech_tools/images/mic.gif';
    if (!final_transcript) {
      showInfo('info_start');
      return;
    }
    showInfo('');


  };

  recognition.onresult = function(event) {
    var interim_transcript = '';
    for (var i = event.resultIndex; i < event.results.length; ++i) {
      if (event.results[i].isFinal) {
        final_transcript += event.results[i][0].transcript;

          Undestanding(final_transcript);
        final_transcript=''
      } else {
        interim_transcript += event.results[i][0].transcript;

      }
    }

        final_transcript = capitalize(final_transcript);

        $('#speech2text').html(linebreak(final_transcript));
        $('#speech2text').html(linebreak(interim_transcript));
  };

}


function start_stop_Button(event) {
  if (recognizing) {
    recognition.stop();
    return;
  }
  start_img=$('#start_img');
  recognition.start();


  //start_timestamp = event.timeStamp;
  start_timestamp =new Date().getTime();
  console.log('start_timestamp ='+start_timestamp);
}

function upgrade() {
  start_button.style.visibility = 'hidden';

}

var two_line = /\n\n/g;
var one_line = /\n/g;
function linebreak(s) {
  return s.replace(two_line, '<p></p>').replace(one_line, '<br>');
}

var first_char = /\S/;
function capitalize(s) {
  return s.replace(first_char, function(m) { return m.toUpperCase(); });
}

function showInfo(s) {
	console.log('showInfo '+s);
}

function Undestanding(speech){

	//start_stop_Button(event); //stop
	console.log('Undestanding(speech)'+speech);

//start_stop_Button(event); //restart
}
