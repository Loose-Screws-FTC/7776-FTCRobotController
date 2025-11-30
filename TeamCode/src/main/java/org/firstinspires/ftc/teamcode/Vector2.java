package org.firstinspires.ftc.teamcode;

public class Vector2 {
    private double X;
    private double Y;

    public Vector2(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    public double GetX() {
        return this.X;
    }

    public double GetY() {
        return this.Y;
    }

    public Vector2 GetVector() {
        return new Vector2(this.X, this.Y);
    }

    public void SetVector(Vector2 Vector) {
        this.X = Vector.GetX();
        this.Y = Vector.GetY();
    }

    public void AddNum(double Num) {
        this.X += Num;
        this.Y += Num;
    }

    public void AddVector2(Vector2 Vector) {
        this.X += Vector.GetX();
        this.Y += Vector.GetY();
    }

    public void SubNum(double Num) {
        this.X -= Num;
        this.Y -= Num;
    }

    public void SubVector2(Vector2 Vector) {
        this.X -= Vector.GetX();
        this.Y -= Vector.GetY();
    }

    public void MultNum(double Num) {
        this.X *= Num;
        this.Y *= Num;
    }

    public void SquareVectWithSign() {
        this.X *= this.X * Math.signum(this.X);
        this.Y *= this.Y * Math.signum(this.Y);
    }

    public void MultVector2(Vector2 Vector) {
        this.X *= Vector.GetX();
        this.Y *= Vector.GetY();
    }

    public void DivNum(double Num) {
        this.X /= Num;
        this.Y /= Num;
    }

    public void DivVector2(Vector2 Vector) {
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
        Clone.DivNum(Magnitude);
        return Clone;
    }

    public double GetDistance(Vector2 Other) {
        Vector2 Clone = new Vector2(this.X, this.Y);
        Clone.SubVector2(Other);
        return Clone.GetMagnitude();
    }
}
