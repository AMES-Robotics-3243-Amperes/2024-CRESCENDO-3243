package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Rate limiter for {@link Translation2d}'s
 * 
 * @author :3
 */
public class TranslationRateLimiter {
  private Translation2d m_previousTranslation = new Translation2d();
  private double m_limit;

  /**
   * Creates a new {@link TranslationRateLimiter}
   * 
   * @param startingTranslation the starting state of the rate limiter
   * 
   * @author :3
   */
  public TranslationRateLimiter(Translation2d startingTranslation, double limit) {
    m_previousTranslation = startingTranslation;
    m_limit = limit;
  }

  public Translation2d calculate(Translation2d input) {
    Translation2d distance = input.minus(m_previousTranslation);
    if (magnitude(distance) > m_limit) {
        Translation2d updated_distance = distance.div(magnitude(distance)).times(m_limit);
        m_previousTranslation = m_previousTranslation.plus(updated_distance);
    } else {
        m_previousTranslation = input;
    }

    return m_previousTranslation;
  }

  /**
   * Returns the magnitude of a {@link Translation2d}
   * 
   * @author :3
   */
  private double magnitude(Translation2d translation) {
    return Math.sqrt(translation.getX() * translation.getX() + translation.getY() * translation.getY());
  }
  /**
   * Resets the rate limited state of the {@link Translation2d}
   * 
   * @param state the new state of the internal rate limited {@link Translation2d}
   */
  public void reset(Translation2d state) {
    m_previousTranslation = state;
  }

  public void changeLimit(double newLimit) {
    m_limit = newLimit;
  }
}
