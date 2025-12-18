import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import DocusaurusChatbot from '../components/DocusaurusChatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <DocusaurusChatbot />
    </>
  );
}
