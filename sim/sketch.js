// broom: 113.8 mm, 18.43 deg

let x = 1300;
let y = 431;
let heading = 0;

function preload() {
    mat = loadImage("mat.png");
}

function straight(distance) {
    endX = x + distance * cos(heading);
    endY = y + distance * sin(heading);
    line(x, y, endX, endY);
    x = endX;
    y = endY;
}

function _curve(radius, angle) {
    pivotX = x + radius * cos(heading + 90);
    pivotY = y + radius * sin(heading + 90);
    circle(pivotX, pivotY, 20);
    if (radius > 0 && angle > 0) {
        start = heading - 90;
        end = heading - 90 + angle;
    } else if (radius > 0 && angle < 0) {
        start = heading - 90 + angle;
        end = heading - 90;
    } else if (radius < 0 && angle > 0) {
        start = heading + 90 - angle;
        end = heading + 90;
    } else if (radius < 0 && angle < 0) {
        start = heading + 90;
        end = heading + 90 - angle;
    }
    arc(pivotX, pivotY, 2 * radius, 2 * radius, start, end);
    x = pivotX + radius * cos(heading + angle + 90);
    y = pivotY + radius * sin(heading + angle + 90);
    heading -= angle;
    console.log(x, y, heading);
}

function setup() {
    angleMode(DEGREES);
    createCanvas(2362, 1143);
    image(mat, 0, 0);
    strokeWeight(5);
    stroke("red");
    noFill();

    straight(-30);
    _curve(-150, -90);
    _curve(-152, -90);
    straight(-300);
    stroke("orange");
    _curve(-95, -90);
    stroke("yellow");
    straight(-40);
    stroke("lime");
    _curve(-160, 90);
    stroke("blue");
    straight(-125);
}
