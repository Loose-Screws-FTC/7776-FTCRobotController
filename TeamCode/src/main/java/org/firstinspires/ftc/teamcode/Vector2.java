package org.firstinspires.ftc.teamcode;

public class Vector2 {
    public double X;
    public double Y;

    public Vector2(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    public Vector2 GetVector() {
        return new Vector2(this.X, this.Y);
    }

    public void Set(Vector2 Vector) {
        this.X = Vector.X;
        this.Y = Vector.Y;
    }

    public void Add(Vector2 Vector) {
        this.X += Vector.X;
        this.Y += Vector.Y;
    }

    public void Sub(Vector2 Vector) {
        this.X -= Vector.X;
        this.Y -= Vector.Y;
    }

    public void Scale(double Num) {
        this.X *= Num;
        this.Y *= Num;
    }

    public void Scale(Vector2 Vector) {
        this.X *= Vector.X;
        this.Y *= Vector.Y;
    }

    public void ComplexMultiply(Vector2 other) {
        double ox = X;
        double oy = Y;
        X = ox * other.X - oy * other.Y;
        Y = ox * other.Y + oy * other.X;
    }

    public double Dot(Vector2 other) {
        return X * other.X + Y * other.Y;
    }

    public void SquareWithSign() {
        this.X *= this.X * Math.signum(this.X);
        this.Y *= this.Y * Math.signum(this.Y);
    }

    public void Pow(double power) {
        this.X = Math.pow(X, power);
        this.Y = Math.pow(Y, power);
    }

    public void Divide(double Num) {
        this.X /= Num;
        this.Y /= Num;
    }

    public void Divide(Vector2 Vector) {
        this.X /= Vector.X;
        this.Y /= Vector.Y;
    }

    public double GetMagnitude() {
        return Math.sqrt(this.X * this.X + this.Y * this.Y);
    }

    public Vector2 Clone() {
        return new Vector2(X, Y);
    }

    public Vector2 GetNormalized() {
        Vector2 clone = Clone();
        double Magnitude = clone.GetMagnitude();
        if (Magnitude == 0) {
            return new Vector2(0, 0);
        }
        clone.Divide(Magnitude);
        return clone;
    }

    public double GetDistance(Vector2 Other) {
        Vector2 clone = Clone();
        clone.Sub(Other);
        return clone.GetMagnitude();
    }

    public static Vector2 AtAngle(double angle) {
        return new Vector2(Math.cos(angle), Math.sin(angle));
    }
}
