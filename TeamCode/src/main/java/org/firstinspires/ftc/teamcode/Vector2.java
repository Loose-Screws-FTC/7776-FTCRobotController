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
        this.X -= Vector.GetX();
        this.Y -= Vector.GetY();
    }

    public void Scale(double Num) {
        this.X *= Num;
        this.Y *= Num;
    }

    public void Scale(Vector2 Vector) {
        this.X *= Vector.GetX();
        this.Y *= Vector.GetY();
    }

    public void SquareWithSign() {
        this.X *= this.X * Math.signum(this.X);
        this.Y *= this.Y * Math.signum(this.Y);
    }

    public void Divide(double Num) {
        this.X /= Num;
        this.Y /= Num;
    }

    public void Divide(Vector2 Vector) {
        this.X /= Vector.GetX();
        this.Y /= Vector.GetY();
    }

    public double GetMagnitude() {
        return Math.sqrt(this.X * this.X + this.Y * this.Y);
    }

    public Vector2 GetNormal() {
        Vector2 Clone = new Vector2(this.X, this.Y);
        double Magnitude = Clone.GetMagnitude();
        if (Magnitude == 0) {
            return new Vector2(0, 0);
        }
        Clone.Divide(Magnitude);
        return Clone;
    }

    public double GetDistance(Vector2 Other) {
        Vector2 Clone = new Vector2(this.X, this.Y);
        Clone.Sub(Other);
        return Clone.GetMagnitude();
    }
}
