import React from 'react';
import './messenger.css';

function Messenger() {
  return (
    <div>
      <br />
      <hr />
      <div className='boxes'>
        <div className='left_box'>
          <input placeholder='Enter your message' />
          <button type='submit'>Send</button>
        </div>

        <div className='vertical-line'>-------------------------------</div>

        <div className='right_box'>
          {/* Renders messages from server */}
        </div>
      </div>
    </div>
  );
}

export default Messenger;
