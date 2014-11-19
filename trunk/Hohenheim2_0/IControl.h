#ifndef ICONTROL_H
#define ICONTROL_H


class IControl {
public:
	IControl(){}
	virtual void initialize() = 0;
	virtual void initializeAuto() = 0;
	virtual void run() = 0;
	virtual void runAuto() = 0;
protected:
	~IControl(){}
	
private:
	
};
#endif
