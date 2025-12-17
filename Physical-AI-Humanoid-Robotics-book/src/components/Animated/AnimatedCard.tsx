import React from 'react';
import { motion } from 'framer-motion';

type AnimatedCardProps = {
  children: React.ReactNode;
  className?: string;
  delay?: number;
  direction?: 'up' | 'down' | 'left' | 'right';
  hoverEffect?: boolean;
  scaleOnHover?: boolean;
  flipOnHover?: boolean;
  rotateOnHover?: boolean;
  shadowOnHover?: boolean;
  customAnimation?: {
    initial?: any;
    animate?: any;
    whileHover?: any;
    transition?: any;
  };
};

const AnimatedCard: React.FC<AnimatedCardProps> = ({
  children,
  className = '',
  delay = 0,
  direction = 'up',
  hoverEffect = true,
  scaleOnHover = true,
  flipOnHover = false,
  rotateOnHover = false,
  shadowOnHover = true,
  customAnimation
}) => {
  const getInitialPosition = () => {
    switch(direction) {
      case 'up': return { y: 50 };
      case 'down': return { y: -50 };
      case 'left': return { x: 50 };
      case 'right': return { x: -50 };
      default: return { y: 50 };
    }
  };

  const getWhileHover = () => {
    if (customAnimation?.whileHover) return customAnimation.whileHover;

    const hoverEffects: any = {};

    if (scaleOnHover) hoverEffects.scale = 1.03;
    if (flipOnHover) hoverEffects.rotateY = 10;
    if (rotateOnHover) hoverEffects.rotate = 2;
    if (shadowOnHover) hoverEffects.y = -5;

    return hoverEffects;
  };

  const animationProps = customAnimation || {
    initial: { opacity: 0, ...getInitialPosition() },
    whileInView: { opacity: 1, x: 0, y: 0 },
    viewport: { once: true, amount: 0.2 },
    transition: {
      duration: 0.6,
      delay,
      ease: [0.25, 0.1, 0.25, 1.0]
    }
  };

  return (
    <motion.div
      {...animationProps}
      whileHover={hoverEffect ? getWhileHover() : undefined}
      className={className}
      style={{
        transition: hoverEffect ? 'transform 0.3s ease, box-shadow 0.3s ease' : undefined,
        cursor: hoverEffect ? 'pointer' : 'default',
        ...animationProps.style
      }}
    >
      {children}
    </motion.div>
  );
};

export default AnimatedCard;