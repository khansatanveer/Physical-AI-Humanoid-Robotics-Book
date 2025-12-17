import React from 'react';
import { motion, cubicBezier } from 'framer-motion';

type AnimatedSectionProps = {
  children: React.ReactNode;
  className?: string;
  id?: string;
};

const container = {
  hidden: { opacity: 0 },
  show: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1
    }
  }
};

const item = {
  hidden: { opacity: 0, y: 20 },
  show: { opacity: 1, y: 0, transition: { duration: 0.5, ease: cubicBezier(0.25, 0.1, 0.25, 1.0) } }
};

const AnimatedSection: React.FC<AnimatedSectionProps> = ({
  children,
  className = '',
  id
}) => {
  return (
    <motion.section
      id={id}
      variants={container}
      initial="hidden"
      whileInView="show"
      viewport={{ once: true, amount: 0.2 }}
      className={className}
    >
      <motion.div variants={item}>
        {children}
      </motion.div>
    </motion.section>
  );
};

export default AnimatedSection;