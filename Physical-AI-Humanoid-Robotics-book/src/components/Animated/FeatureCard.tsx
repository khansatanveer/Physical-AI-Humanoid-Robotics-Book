import React from 'react';
import { motion } from 'framer-motion';
import clsx from 'clsx';

type FeatureCardProps = {
  title: string;
  description: React.ReactNode;
  icon?: React.ReactNode;
  imageUrl?: string;
  link?: string;
  delay?: number;
  index?: number;
  className?: string;
  variant?: 'default' | 'elevated' | 'floating' | 'glow';
};

const FeatureCard: React.FC<FeatureCardProps> = ({
  title,
  description,
  icon,
  imageUrl,
  link,
  delay = 0,
  index = 0,
  className = '',
  variant = 'default'
}) => {
  const getVariantStyles = () => {
    switch (variant) {
      case 'elevated':
        return 'shadow-lg hover:shadow-xl transition-all duration-300';
      case 'floating':
        return 'shadow-md hover:shadow-lg hover:-translate-y-1 transition-all duration-300';
      case 'glow':
        return 'shadow-lg hover:shadow-[0_0_20px_rgba(26,95,180,0.3)] transition-all duration-300';
      default:
        return 'shadow-md hover:shadow-lg transition-all duration-300';
    }
  };

  const getAnimationVariants = () => {
    return {
      initial: { opacity: 0, y: 30, scale: 0.95 },
      animate: { opacity: 1, y: 0, scale: 1 },
      whileHover: {
        y: -5,
        scale: 1.02,
        transition: { duration: 0.2 }
      }
    };
  };

  const cardContent = (
    <div className={clsx(
      'card',
      'h-full',
      'flex',
      'flex-col',
      'items-center',
      'text-center',
      'p-6',
      'rounded-xl',
      'bg-white',
      'dark:bg-gray-800',
      'border',
      'border-gray-200',
      'dark:border-gray-700',
      getVariantStyles(),
      className
    )}>
      {icon && (
        <div className="mb-4 flex justify-center">
          <div className="w-16 h-16 flex items-center justify-center text-3xl">
            {icon}
          </div>
        </div>
      )}

      {imageUrl && (
        <div className="mb-4 flex justify-center">
          <img
            src={imageUrl}
            alt={title}
            className="w-16 h-16 object-contain"
          />
        </div>
      )}

      <h3 className="text-xl font-semibold text-gray-900 dark:text-white mb-2">
        {title}
      </h3>

      <p className="text-gray-600 dark:text-gray-300 text-base flex-grow">
        {description}
      </p>
    </div>
  );

  return (
    <motion.div
      initial={{ opacity: 0, y: 30 }}
      animate={{ opacity: 1, y: 0 }}
      whileHover={{
        y: -8,
        scale: 1.03,
        transition: { duration: 0.2 }
      }}
      transition={{
        duration: 0.6,
        delay: delay + (index * 0.1),
        ease: [0.25, 0.1, 0.25, 1.0]
      }}
      className={link ? 'cursor-pointer' : ''}
    >
      {link ? (
        <a href={link} className="block h-full no-underline">
          {cardContent}
        </a>
      ) : (
        cardContent
      )}
    </motion.div>
  );
};

export default FeatureCard;