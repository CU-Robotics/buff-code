class velPID {
    public:
    velPID(int range, float p, float i, float d);
    int calculate(int currVel);
    private:

};