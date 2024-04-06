package frc.robot.commands.Interpolation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShotCalculator
{
  private double m_forwardDistance = 0, m_forwardVelocity = 0;
  private double m_verticalDistance = 0, m_verticalVelocity = 0;
  private double m_horizontalDistance = 0, m_horizontalVelocity = 0;
  
  private double m_ballSpeed = 18;
  private double m_ballGravity = -9.8;
  private double m_maxShotTime = 30;
  
  private boolean straightShot = true;
  private boolean shotExists = true;
  
  public void setPositions(double m_forwardDistance, double m_verticalDistance, double m_horizontalDistance)
  {
    this.m_forwardDistance = m_forwardDistance;
    this.m_verticalDistance = m_verticalDistance;
    this.m_horizontalDistance = m_horizontalDistance; 
  }

  public void setPositions(Translation2d robotPos, Translation2d targetPos) {
    this.m_forwardDistance = targetPos.getX()-robotPos.getX();
    // ShotCalculator.m_verticalDistance = 1.56;
    this.m_horizontalDistance = targetPos.getY()-robotPos.getY();
    //ShotCalculator.m_verticalDistance = InterpolatingTable.get(Math.hypot(Math.abs(m_forwardDistance), Math.abs(m_horizontalDistance))).pivotAngleRotations;
    double x = Math.hypot(Math.abs(m_forwardDistance), Math.abs(m_horizontalDistance));
    //this.m_verticalDistance = (1 / (1.25*(x-.1))) + .8;
    this.m_verticalDistance = InterpolatingTable.get(x).pivotAngleRotations;
    //ShotCalculator.m_verticalDistance = 1;
  }
  
  public void setVelocities(double m_forwardVelocity, double m_verticalVelocity, double m_horizontalVelocity) 
  {
    this.m_forwardVelocity = -m_forwardVelocity;
    this.m_verticalVelocity = -m_verticalVelocity;
    this.m_horizontalVelocity = -m_horizontalVelocity;
  }
  
  public void setSpeed(double m_ballSpeed) 
  {
    this.m_ballSpeed = m_ballSpeed;
  }
  
  public void isStraightShot(boolean straightShot) {
    this.straightShot = straightShot;
  }
 
  
  public double[] shoot()
  {
    shotExists = true;
    
    double timeInterval1 = 0;
    double timeInterval2 = 0;

    while (test(timeInterval2) < 0)
    {
        timeInterval2 += 1;
        if (timeInterval2 > m_maxShotTime)
        {
            shotExists = false;
            break;
        }
    }
    if (!shotExists)
    {
        return new double[] {0, 0, 0};
    }

    double Time = bisectionMethod(timeInterval1, timeInterval2);

    double shortXComponent = (m_forwardDistance / Time) - m_forwardVelocity;
    double shortZComponent = (m_horizontalDistance / Time) - m_horizontalVelocity;
    double shortYComponent = Math.sqrt(m_ballSpeed*m_ballSpeed - shortXComponent*shortXComponent - shortZComponent*shortZComponent);
    

    timeInterval1 = Time + .0001;
    timeInterval2 = Time + .0001;

    while (test(timeInterval2) > 0)
    {
        timeInterval2 += 1;
        if (timeInterval2 > m_maxShotTime)
        {
            shotExists = false;
            break;
        }
    }
    if (!shotExists)
    {
        return new double[] {0, 0, 0};
    }

    Time = bisectionMethod(timeInterval2, timeInterval1);
    
    double longXComponent = (m_forwardDistance / Time);
    double longZComponent = (m_horizontalDistance / Time);
    double longYComponent = Math.sqrt(m_ballSpeed*m_ballSpeed - longXComponent*longXComponent - longZComponent*longZComponent);
    
  
    return straightShot ? new double[] {shortXComponent, shortYComponent, shortZComponent} : new double[]{longXComponent, longYComponent, longZComponent};
  }
  
  private double bisectionMethod(double time1, double time2)
  {
    double negativeTimeInterval = time1;
    double positiveTimeInterval = time2;

    double midpoint = (negativeTimeInterval + positiveTimeInterval) * .5;
    double guess = test(midpoint);

    while (Math.abs(guess) > .001f)
    {
        if (guess > 0) positiveTimeInterval = midpoint;
        else negativeTimeInterval = midpoint;

        midpoint = (negativeTimeInterval + positiveTimeInterval) * .5;
        guess = test(midpoint);
    }
    return midpoint;
  }
  
  private double test(double timeInput)
  {
    double d = m_ballSpeed * timeInput;
    double gPos = getGPos(timeInput) - m_verticalDistance;
    double forward = m_forwardDistance - (m_forwardVelocity * timeInput); 
    double vertical = gPos - (m_verticalVelocity * timeInput);
    double horizontal = m_horizontalDistance - (m_horizontalVelocity * timeInput);

    return d - Math.sqrt((forward*forward) + (vertical*vertical) + (horizontal*horizontal));
  }

  public Translation3d getRelativePos() {
    return new Translation3d(m_forwardDistance, m_horizontalDistance, m_verticalDistance);
  }

  private double getGPos(double time)
  {
    return (m_ballGravity * (time * time) * .5);
  }
}