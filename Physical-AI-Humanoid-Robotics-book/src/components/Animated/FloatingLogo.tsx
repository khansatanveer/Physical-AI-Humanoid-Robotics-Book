import React from 'react';
import { motion } from 'framer-motion';

type FloatingLogoProps = {
  src?: string;
  SvgComponent?: React.ComponentType<React.ComponentProps<'svg'>>;
  alt?: string;
  size?: 'sm' | 'md' | 'lg';
  className?: string;
};

const FloatingLogo: React.FC<FloatingLogoProps> = ({
  src,
  SvgComponent,
  alt,
  size = 'md',
  className = ''
}) => {
  const sizeClasses = {
    sm: 'w-16 h-16',
    md: 'w-24 h-24',
    lg: 'w-32 h-32'
  };

  return (
    <motion.div
      className={`${sizeClasses[size]} ${className}`}
      animate={{
        y: [-10, 10, -10],
      }}
      transition={{
        duration: 3,
        repeat: Infinity,
        ease: "easeInOut"
      }}
    >
      {SvgComponent ? (
        <SvgComponent className="w-full h-full object-contain" role="img" />
      ) : (
        <img src={src} alt={alt} className="w-full h-full object-contain" />
      )}
    </motion.div>
  );
};

export default FloatingLogo;