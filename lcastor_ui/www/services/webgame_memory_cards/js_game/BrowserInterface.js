/**
 * Author: Roberto Pinillos
 * Project: Memory Workout
 * Description: The game interface
 */

/**
 * @TODO Refactor code.
 */

//DATA

var Topic= 'farm'
var data_fruits=[
    {
        "speech": "apple", 
        "image": "./images/fruits/apple.jpg" 
    },
    {
        "speech": "fig", 
        "image": "./images/fruits/fig.jpg" 
    },
    {
        "speech": "grape", 
        "image": "./images/fruits/grape.jpg" 
    },
    {
        "speech": "kiwi", 
        "image": "./images/fruits/kiwi.jpg" 
    },
    {
        "speech": "lemon", 
        "image": "./images/fruits/lemon.jpg" 
    },
    {
        "speech": "lime", 
        "image": "./images/fruits/lime.jpg" 
    },
    {
        "speech": "mango", 
        "image": "./images/fruits/mango.jpg" 
    },
    {
        "speech": "melon", 
        "image": "./images/fruits/melon.jpg" 
    },
    {
        "speech": "orange", 
        "image": "./images/fruits/orange.jpg" 
    },
    {
        "speech": "peach", 
        "image": "./images/fruits/peach.jpg" 
    },
    {
        "speech": "pear", 
        "image": "./images/fruits/pear.jpg" 
    },
    {
        "speech": "pinapple", 
        "image": "./images/fruits/pinapple.jpg" 
    },
    {
        "speech": "plum", 
        "image": "./images/fruits/plum.jpg" 
    },
    {
        "speech": "raspberry", 
        "image": "./images/fruits/raspberry.jpg" 
    },
    {
        "speech": "strawberry", 
        "image": "./images/fruits/strawberry.jpg" 
    }


];

var data_farm=[
    {
        "speech": "barn", 
        "image": "./images/farm/barn.png" 
    },
    {
        "speech": "chick", 
        "image": "./images/farm/chick.png" 
    },
    {
        "speech": "chicken", 
        "image": "./images/farm/chicken.png" 
    },
    {
        "speech": "cow", 
        "image": "./images/farm/cow.png" 
    },
    {
        "speech": "duck", 
        "image": "./images/farm/duck.png" 
    },
    {
        "speech": "farmer", 
        "image": "./images/farm/farmer.png" 
    },
    {
        "speech": "horse", 
        "image": "./images/farm/horse.png" 
    },
    {
        "speech": "pig", 
        "image": "./images/farm/pig.png" 
    },
    {
        "speech": "rooster", 
        "image": "./images/farm/rooster.png" 
    },
    {
        "speech": "sheep", 
        "image": "./images/farm/sheep.png" 
    },
    {
        "speech": "tools", 
        "image": "./images/farm/tools.png" 
    },
    {
        "speech": "tractor", 
        "image": "./images/farm/tractor.png" 
    },
    {
        "speech": "trees", 
        "image": "./images/farm/trees.png" 
    },
    {
        "speech": "wheelbarrow", 
        "image": "./images/farm/wheelbarrow.png" 
    },
    {
        "speech": "wildpig", 
        "image": "./images/farm/wildpig.png" 
    }


];

var topic_cards=null;

var fixed_width_screen=800;
var fixed_height_screen=580;

function getRadioVal(form, name) {
    var val;
    // get list of radio buttons with specified name
    var radios = form.elements[name];
    
    // loop through list of radio buttons
    for (var i=0, len=radios.length; i<len; i++) {
        if ( radios[i].checked ) { // radio checked?
            val = radios[i].value; // if so, hold its value in val
            break; // and break out of for loop
        }
    }
    return val; // return value of checked radio or undefined if none checked
}
//(function($) {

function Start_game(){

  if (Topic=='fruits') topic_cards=data_fruits;
  else if (Topic=='farm') topic_cards=data_farm;

  var memory = Object.create(MemoryGame);

  // Handle clicking on card
  var handleFlipCard = function (event) {

    event.preventDefault();

    var status = memory.play(this.index);
    console.log(status);

    if (status.code != 0 ) {
      this.classList.toggle('clicked');
	speechMessage('clicked');
    }

    if (status.code == 2 ) { // Match
        speechMessage('match');
    }
    if (status.code == 3 ) {
      
      setTimeout(function(){
          speechMessage('fail');
      }, 800);
      setTimeout(function () {
        var childNodes = document.getElementById('memory--cards').childNodes;
        childNodes[status.args[0]].classList.remove('clicked');
        childNodes[status.args[1]].classList.remove('clicked');
      }.bind(status), 1500);

    }
    else if (status.code == 4) {


      var score = parseInt(((memory.attempts - memory.mistakes) / memory.attempts) * 100, 10);
			EndGameMessage(score);
/*      var message = getEndGameMessage(score);

      document.getElementById('memory--end-game-message').textContent = message;
      document.getElementById('memory--end-game-score').textContent =
          'Score: ' + score + ' / 100';

      document.getElementById("memory--end-game-modal").classList.toggle('show');
*/
    }

  };



  // Build grid of cards
  var buildLayout =  function (cards, rows, columns) {
    if (!cards.length) {
      return;
    }

    var memoryCards = document.getElementById("memory--cards");
    var index = 0;
		var maxwidth=document.getElementById('memory--app-container').offsetWidth;

		if (maxwidth==0)maxwidth=fixed_width_screen;
		var maxheight=document.getElementById('memory--app-container').offsetHeight ;
		if (maxheight==0)maxheight=fixed_height_screen;
    var cardMaxWidth = maxwidth / columns;
    var cardHeightForMaxWidth = cardMaxWidth * (3 / 4);

    var cardMaxHeight =maxheight / rows;
    var cardWidthForMaxHeight = cardMaxHeight * (4 / 3);

    // Clean up. Remove all child nodes and card clicking event listeners.
    while (memoryCards.firstChild) {
      memoryCards.firstChild.removeEventListener('click', handleFlipCard);
      memoryCards.removeChild(memoryCards.firstChild);
    }

    for (var i = 0; i < rows; i++) {
      for (var j = 0; j < columns; j++) {
        // Use cloneNode(true) otherwise only one node is appended
        memoryCards.appendChild(buildCardNode(
            index, cards[index].value, cards[index].isRevealed,
            (100 / columns) + "%", (100 / rows) + "%"));
        index++;
      }
    }

    // Resize cards to fit in viewport

    if (cardMaxHeight > cardHeightForMaxWidth) {

      // Update height
      memoryCards.style.height = (cardHeightForMaxWidth * rows) + "px";
      memoryCards.style.width = document.getElementById('memory--app-container').offsetWidth + "px";
      memoryCards.style.top = ((cardMaxHeight * rows - (cardHeightForMaxWidth * rows)) / 2) + "px";
    }
    else {
      // Update Width


      memoryCards.style.width = (cardWidthForMaxHeight * columns) + "px";
      memoryCards.style.height = document.getElementById('memory--app-container').offsetHeight + "px";
      memoryCards.style.top = 0;
    }
		

  };

  // Update on resize
  window.addEventListener('resize', function() {
    buildLayout(memory.cards, memory.settings.rows, memory.settings.columns);

  }, true);

  // Build single card
  var buildCardNode = function (index, value, isRevealed, width, height) {
    var flipContainer = document.createElement("li");
    var flipper = document.createElement("div");
    var front = document.createElement("a");
    var back = document.createElement("a");

    flipContainer.index = index;
    flipContainer.style.width = width;
    flipContainer.style.height = height;
    flipContainer.classList.add("flip-container");
    if (isRevealed) {
      flipContainer.classList.add("clicked");
    }

    flipper.classList.add("flipper");
    front.classList.add("front");
		front.style.backgroundImage = "url(./images/card.svg)";

    front.setAttribute("href", "#");
    back.classList.add("back");
    //back.classList.add("card-" + value);
		back.style.backgroundImage = "url("+topic_cards[value]["image"]+")";

    back.setAttribute("href", "#");

    flipper.appendChild(front);
    flipper.appendChild(back);
    flipContainer.appendChild(flipper);

    flipContainer.addEventListener('click', handleFlipCard);

    return flipContainer;
  };


    var selectWidget = document.getElementById("level-choice");
    var grid = getRadioVal(selectWidget, 'memory--settings-grid');

    var gridValues = grid.split('x');
    var cards = memory.initialize(Number(gridValues[0]), Number(gridValues[1]));
 
    if (cards) {

      console.log('before buildLayout');
      buildLayout(memory.cards, memory.settings.rows, memory.settings.columns);
      console.log('after buildLayout');
    }





		

/*  var speechMessage = function(text) {
    var message = text;

    var popup=document.getElementById("dialog-popup");
    document.getElementById("dialog-message").textContent = message;
    popup.classList.toggle('show');

      setTimeout(function(){
          popup.classList.toggle('show');
      }, 800);
  }
*/


}
//}(MemoryGame);