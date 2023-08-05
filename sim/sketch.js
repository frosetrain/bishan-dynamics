// broom: 113.8 mm, 18.43 deg

let x = 138;
let y = 1052;
let heading = 0;
let frames = [];
let pivotX = x;
let pivotY = y;

// prettier-ignore
var movements = [
    // function() { ccurve((160 * 2) / 3, 47.25); },
    // function() { ccurve((160 * 2) / 3, -47.25); },
    // function() { tp(209, 344) },
    // function() { turn(30); },
    // function() { straight(165); },
    // function() { straight(-165); },
    // function() { turn(-35); },
    // function() { straight(105); },
    // function() { turn(150); },
    // function() { straight(235); },
    // function() { turn(-55); },
    function() { tp(1396, 431) },
    function() { straight(50); },
    function() { ccurve((160 * 2) / 3, -90); },
    function() { straight(50); },
    function() { ccurve((160 * 2) / 3, 180); },
    function() { straight(40); },
    function() { turn(90); },
    function() { straight(70); },
    function() { ccurve((160 * 2) / 3, 90); },
    function() { ccurve((160 * 2) / 3, -180); },
    function() { straight(60); },
    function() { turn(-90); },
    function() { turn(90); },
    function() { ccurve(160, -40); },
    function() { ccurve(160, 40); },
    function() { straight(80); },
    function() { straight(-200); },
    function() { turn(90); },
]

function preload() {
    mat = loadImage("mat.png");
}

function setup() {
    createCanvas(2362, 1143);
    image(mat, 0, 0);
    angleMode(DEGREES);

    for (let mid = 0; mid < movements.length; ++mid) {
        movements[mid]();
    }
    console.log(frames);
}

function straight(distance) {
    // x = r cos theta
    // y = r sin theta
    if (distance > 0) {
        for (let i = 0; i < distance; i += 4) {
            frames.push([x + i * cos(heading), y - i * sin(heading), heading]);
        }
    } else {
        for (let i = 0; i > distance; i -= 4) {
            frames.push([x + i * cos(heading), y - i * sin(heading), heading]);
        }
    }
    x = frames[frames.length - 1][0];
    y = frames[frames.length - 1][1];
}

function turn(angle) {
    if (angle > 0) {
        for (let i = 0; i < angle; ++i) {
            frames.push([x, y, heading - i]);
        }
    } else {
        for (let i = 0; i > angle; --i) {
            frames.push([x, y, heading - i]);
        }
    }
    heading = frames[frames.length - 1][2];
}

function ccurve(radius, angle) {
    console.log("curve", x, y, radius, heading);
    if (angle > 0) {
        pivotX = x + radius * cos(heading - 90);
        pivotY = y + radius * sin(heading - 90);
        for (let i = 0; i < angle; ++i) {
            frames.push([
                pivotX - radius * cos(heading - 90 + i),
                pivotY - radius * sin(heading - 90 + i),
                heading - i,
            ]);
        }
    } else {
        console.log("neggative");
        pivotX = x - radius * cos(90 - heading);
        pivotY = y - radius * sin(90 - heading);
        for (let i = 0; i > angle; --i) {
            frames.push([
                pivotX + radius * cos(-heading + 90 + i),
                pivotY + radius * sin(-heading + 90 + i),
                heading - i,
            ]);
        }
    }
    x = frames[frames.length - 1][0];
    y = frames[frames.length - 1][1];
    heading = frames[frames.length - 1][2];
}

function tp(tx, ty) {
    x = tx;
    y = ty;
}

let fid = 0;
function draw() {
    x = frames[fid][0];
    y = frames[fid][1];
    heading = frames[fid][2];
    no();
    fid++;
}

function no() {
    push();
    clear();
    image(mat, 0, 0);
    translate(x, y);
    rotate(90 - heading);
    fill("#FAC80A");
    rect(-44, -44, 88, 56);
    fill("#68C3E2");
    rect(-68, -60, 24, 72);
    rect(44, -60, 24, 72);
    fill("black");
    rect(-92, -28, 16, 56);
    rect(76, -28, 16, 56);
    fill("gray");
    square(-28, -76, 24);
    square(4, -76, 24);
    fill("red");
    circle(113.8 * cos(71.57), -113.8 * sin(71.57), 16);
    pop();
}
