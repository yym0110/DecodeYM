package org.firstinspires.ftc.teamcode.utils;

public class Complex {

    private double re;
    private double im;
    private double magCache = -1;
    private double thetaCache;

    public Complex(double real, double imag) {
        re = real;
        im = imag;
        magCache = Math.sqrt(re * re + im * im);
        thetaCache = Math.atan2(im, re);
    }

    public Complex(Complex z) {
        re = z.re;
        im = z.im;
        magCache = z.magCache;
        thetaCache = z.thetaCache;
    }

    public void add(Complex b) { re += b.re; im += b.im; magCache = -1; thetaCache = -10; }

    public void addReal(double a) { re += a; magCache = -1; thetaCache = -10; }

    public void addImag(double a) { im += a; magCache = -1; thetaCache = -10; }

    public void multReal(double m) { re *= m; im *= m; magCache *= m; }

    public void multImag(double m) {
        double re1 = -im * m; double im1 = re * m;
        re = re1; im = im1;
        magCache = -1; thetaCache = -10;
    }

    public void multZ(Complex z) {
        double re1 = re * z.re - im * z.im;
        double im1 = im * z.re + re * z.im;
        re = re1; im = im1;
    }

    public void nRoot(double n) {
        thetaCache = theta() / n;
        magCache = Math.pow(mag(), 1 / n);
        re = magCache * Math.cos(thetaCache);
        im = magCache * Math.sin(thetaCache);
    }

    /**
     * if theta returned is -10, then the mag is 0
     * @return theta
     */
    public double theta() {
        if (thetaCache != -10) return thetaCache;
        if (re == 0 && im == 0) return thetaCache;
        thetaCache = Math.atan2(im, re);
        return thetaCache;
    }

    public double mag() {
        if (magCache != -1) return magCache;
        magCache = Math.sqrt(re * re + im * im);
        return magCache;
    }

    public double real() { return re; }
    public double imag() { return im; }

    public void conjugate() {
        im = -im;
    }

    public static Complex add(Complex a, Complex b) {
        return new Complex(a.re + b.re, a.im + b.im);
    }

    public static Complex divide(Complex a1, Complex b1) {
        Complex a = new Complex(a1);
        Complex b = new Complex(b1);
        double denom = b.mag();
        b.conjugate();
        a.multZ(b);
        a.multReal(1 / denom);
        return a;
    }

    public static Complex nRoot(double n, Complex z1) {
        Complex z = new Complex(z1);
        z.nRoot(n);
        return z;
    }

}
