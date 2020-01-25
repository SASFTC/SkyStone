package org.firstinspires.ftc.team17156;

class Vector3{
    private int red;
    private int green;
    private int blue;
    private String hex;

    public Vector3(int red, int green, int blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;

        if ( 0 >  this.red  || this.red >= 255){
            System.out.println("red value should be between 0 and 255!");

        }

        if ( 0 >  this.green  ||  this.green >= 255){
            System.out.println("green value should be between 0 and 255!");
        }

        if ( 0 > this.blue  || this.blue >= 255){
            System.out.println("blue value should be between 0 and 255!");
        }
    }

    public int getRed() {
        return red;
    }

    public int getGreen() {
        return green;
    }

    public int getBlue() {
        return blue;
    }

    public String getHex() {
        hex = String.format("#%02x%02x%02x", this.red, this.green, this.blue);
        return hex;
    }
}