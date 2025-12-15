import React from 'react';
import RTLHandler from './RTLHandler';

const Root = ({children}) => {
  return (
    <>
      <RTLHandler />
      {children}
    </>
  );
};

export default Root;